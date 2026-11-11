#!/usr/bin/env python3
# Copyright 2026 A Gudamu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deterministic offline evaluation of kf_hungarian_tracker (no executor, no wall clock).

The tracker node is instantiated and its detection callback is called directly with detections
that are already in the global frame, so only the tracking logic is measured (association,
filtering, birth/death, publication filters). Many seeds run in seconds.

  python3 offline_eval.py [--seeds 20] [--pmiss 0.1] [--fp 0.2] [--noise 0.05] [--set key=value]
"""

import argparse
import copy
import json
import math
import sys
import collections
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from kf_hungarian_tracker.kf_hungarian_node import KFHungarianTracker

GATE, MOVING = 1.0, 0.2


class Walker:
    def __init__(self, rng):
        self.rng = rng
        self.speed = float(rng.uniform(0.5, 1.6))
        self.p = np.array([rng.uniform(-6, 6), rng.uniform(-6, 6)])
        self.v = np.zeros(2)
        self.pause = 0.0
        self.goal = np.array([rng.uniform(-6, 6), rng.uniform(-6, 6)])

    def step(self, dt):
        if self.pause > 0:
            self.pause -= dt
            self.v = np.zeros(2)
            return
        d = self.goal - self.p
        n = np.linalg.norm(d)
        if n < self.speed * dt:
            self.p = self.goal.copy()
            self.v = np.zeros(2)
            if self.rng.random() < 0.4:
                self.pause = float(self.rng.uniform(2, 6))
            self.goal = np.array([self.rng.uniform(-6, 6), self.rng.uniform(-6, 6)])
            return
        self.v = d / n * self.speed
        self.p = self.p + self.v * dt


def run(seed, a, overrides):
    rng = np.random.default_rng(seed)
    node = KFHungarianTracker()
    node.transform_to_global_frame = False
    for k, v in overrides.items():
        setattr(node, k, v)
    out = []
    node.tracker_obstacle_pub.get_subscription_count = lambda: 1
    node.tracker_obstacle_pub.publish = lambda m: out.append(m)
    people = [Walker(rng) for _ in range(a.people)]
    static = [np.array([rng.uniform(-6, 6), rng.uniform(-6, 6)]) for _ in range(a.static)]
    dt = 1.0 / a.rate
    S = collections.defaultdict(float)
    pos_err, vel_err = [], []
    last_id, moving_since = {}, {}
    switches = 0
    in_prev = {}
    for k in range(int(a.duration * a.rate)):
        t = 100.0 + k * dt
        for p in people:
            p.step(dt)
        objs = [(i, p.p.copy(), p.v.copy()) for i, p in enumerate(people)] + [
            (100 + i, s, np.zeros(2)) for i, s in enumerate(static)
        ]
        det = ObstacleArray()
        det.header.frame_id = "odom"
        det.header.stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)
        for oid, p, v in objs:
            if np.linalg.norm(p) > a.range or rng.random() < a.pmiss:
                continue
            d = Obstacle()
            n = rng.normal(0, a.noise, 2)
            d.position.x, d.position.y, d.position.z = float(p[0] + n[0]), float(p[1] + n[1]), 0.8
            d.size.x = d.size.y = 0.5
            d.size.z = 1.7
            d.orientation.w = 1.0
            det.obstacles.append(d)
        for _ in range(rng.poisson(a.fp)):
            d = Obstacle()
            ang = rng.uniform(0, 2 * math.pi)
            r = rng.uniform(1, a.range)
            d.position.x, d.position.y, d.position.z = (
                float(r * math.cos(ang)),
                float(r * math.sin(ang)),
                0.5,
            )
            d.size.x = d.size.y = d.size.z = 0.4
            d.orientation.w = 1.0
            det.obstacles.append(d)
        out.clear()
        node.callback(det)
        published = out[0].obstacles if out else []
        K = [
            (
                bytes(o.uuid.uuid),
                np.array([o.position.x, o.position.y]),
                np.array([o.velocity.x, o.velocity.y]),
            )
            for o in published
        ]
        pairs = sorted(
            (
                (np.linalg.norm(tp - kp), i, j)
                for i, (_, tp, _) in enumerate(objs)
                for j, (_, kp, _) in enumerate(K)
            ),
            key=lambda x: x[0],
        )
        tu, ku, match = set(), set(), {}
        for d_, i, j in pairs:
            if d_ > GATE:
                break
            if i in tu or j in ku:
                continue
            tu.add(i)
            ku.add(j)
            match[i] = (j, d_)
        for i, (tid, tp, tv) in enumerate(objs):
            inr = np.linalg.norm(tp) <= a.range
            moving = np.linalg.norm(tv) > MOVING and tid < 100
            if inr and not in_prev.get(tid, False) and tid < 100:
                last_id.pop(tid, None)  # a new visit may legitimately get a new id
            in_prev[tid] = inr
            if moving and inr:
                moving_since.setdefault(tid, t)
            else:
                moving_since.pop(tid, None)
            if not inr:
                continue
            if not moving:
                S["static_frames"] += 1
                S["static_reported"] += i in match
                continue
            S["moving_frames"] += 1
            settled = t - moving_since[tid] > 1.0
            S["settled_frames"] += settled
            if i in match:
                j, d_ = match[i]
                S["moving_tracked"] += 1
                S["settled_tracked"] += settled
                pos_err.append(d_)
                vel_err.append(float(np.linalg.norm(K[j][2] - tv)))
                if tid in last_id and last_id[tid] != K[j][0]:
                    switches += 1
                last_id[tid] = K[j][0]
        S["ghosts"] += len(K) - len(ku)
        S["msgs"] += 1
    node.destroy_node()
    pe, ve = np.array(pos_err or [np.nan]), np.array(vel_err or [np.nan])
    minutes = max(S["moving_frames"] * dt / 60.0, 1e-9)
    return dict(
        coverage=S["moving_tracked"] / max(S["moving_frames"], 1),
        coverage_after_1s=S["settled_tracked"] / max(S["settled_frames"], 1),
        pos_rmse=float(np.sqrt(np.mean(pe**2))),
        vel_rmse=float(np.sqrt(np.mean(ve**2))),
        id_switches_per_person_min=switches / minutes,
        static_reported_frac=S["static_reported"] / max(S["static_frames"], 1),
        ghosts_per_frame=S["ghosts"] / S["msgs"],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--noise", type=float, default=0.05)
    ap.add_argument("--pmiss", type=float, default=0.1)
    ap.add_argument("--fp", type=float, default=0.2)
    ap.add_argument("--range", type=float, default=8.0)
    ap.add_argument("--people", type=int, default=6)
    ap.add_argument("--static", type=int, default=6)
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--label", default="")
    a = ap.parse_args()
    ov = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        ov[k] = json.loads(v)
    rclpy.init()
    R = [run(s, a, copy.deepcopy(ov)) for s in range(1, a.seeds + 1)]
    rclpy.shutdown()
    keys = list(R[0].keys())
    mean = {k: float(np.mean([r[k] for r in R])) for k in keys}
    sem = {
        k: float(np.std([r[k] for r in R], ddof=1) / math.sqrt(len(R))) if len(R) > 1 else 0.0
        for k in keys
    }
    print(
        "%-26s " % (a.label or " ".join(sys.argv[1:]))[:26]
        + "  ".join("%s %.3f±%.3f" % (k[:12], mean[k], sem[k]) for k in keys)
    )
    return mean


if __name__ == "__main__":
    main()
