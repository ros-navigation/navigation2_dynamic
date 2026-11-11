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

"""Scores /tracking (ObstacleArray) against /truth. Writes a JSON summary on exit.

For every tracking message the truth sample with the same stamp is used (the tracker keeps the
detection stamp). Tracks are matched to truth objects greedily by distance (gate 1.0 m).
"""

import json
import sys
import time
import collections
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_dynamic_msgs.msg import ObstacleArray

GATE = 1.0
MOVING = 0.2  # m/s, truth speed above this = "moving"


def key(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Eval(Node):
    def __init__(self, out, duration):
        super().__init__("eval")
        self.out, self.duration = out, duration
        self.truth = collections.OrderedDict()
        self.create_subscription(ObstacleArray, "truth", self.on_truth, 50)
        self.create_subscription(ObstacleArray, "tracking", self.on_track, 50)
        self.t0 = None
        self.S = collections.defaultdict(float)  # accumulators
        self.ids = collections.defaultdict(list)  # truth id -> sequence of track uuids
        self.first_seen, self.first_tracked = {}, {}
        self.pos_err, self.vel_err, self.lat = [], [], []
        self.moving_since = {}
        self.create_timer(0.5, self.check)

    def check(self):
        if self.t0 is not None and time.monotonic() - self.t0 > self.duration + 3.0:
            self.finish()

    def on_truth(self, m):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.truth[key(m.header.stamp)] = m
        while len(self.truth) > 100:
            self.truth.popitem(last=False)

    def on_track(self, m):
        k = key(m.header.stamp)
        tr = self.truth.get(k)
        self.S["track_msgs"] += 1
        if tr is None:
            self.S["no_truth_for_stamp"] += 1
            return
        self.lat.append((self.get_clock().now().nanoseconds - k) / 1e9)
        if m.header.frame_id != "odom":
            self.S["wrong_frame"] += 1
        T = [
            (
                o.uuid.uuid[-1],
                np.array([o.position.x, o.position.y]),
                np.array([o.velocity.x, o.velocity.y]),
                o.score > 0.5,
            )
            for o in tr.obstacles
        ]
        K = [
            (
                bytes(o.uuid.uuid),
                np.array([o.position.x, o.position.y]),
                np.array([o.velocity.x, o.velocity.y]),
            )
            for o in m.obstacles
        ]
        pairs = sorted(
            (
                (np.linalg.norm(tp - kp), i, j)
                for i, (_, tp, _, _) in enumerate(T)
                for j, (_, kp, _) in enumerate(K)
            ),
            key=lambda x: x[0],
        )
        tu, ku, match = set(), set(), {}
        for d, i, j in pairs:
            if d > GATE:
                break
            if i in tu or j in ku:
                continue
            tu.add(i)
            ku.add(j)
            match[i] = (j, d)
        now = k / 1e9
        for i, (tid, tp, tv, in_range) in enumerate(T):
            speed = np.linalg.norm(tv)
            moving = speed > MOVING and tid < 100
            if moving and in_range:
                self.moving_since.setdefault(tid, now)
            else:
                self.moving_since.pop(tid, None)
            if not in_range:
                continue
            if tid >= 100 or not moving:  # static object or a person standing still
                self.S["static_frames"] += 1
                if i in match:
                    self.S["static_reported"] += 1  # reported as a (moving) tracked obstacle
                continue
            self.S["moving_frames"] += 1
            if now - self.moving_since[tid] > 1.0:
                self.S["moving_frames_settled"] += 1  # has been moving and visible > 1 s
            if i in match:
                j, d = match[i]
                self.S["moving_tracked"] += 1
                if now - self.moving_since[tid] > 1.0:
                    self.S["moving_tracked_settled"] += 1
                self.pos_err.append(d)
                self.vel_err.append(float(np.linalg.norm(K[j][2] - tv)))
                seq = self.ids[tid]
                if not seq or seq[-1] != K[j][0]:
                    seq.append(K[j][0])
        self.S["ghost_tracks"] += len(K) - len(
            ku
        )  # published tracks with no truth object within the gate
        self.S["tracks_published"] += len(K)

    def finish(self):
        S = self.S
        mf = max(S["moving_frames"], 1)
        ms = max(S["moving_frames_settled"], 1)
        pe, ve, lat = (
            np.array(self.pos_err or [np.nan]),
            np.array(self.vel_err or [np.nan]),
            np.array(self.lat or [np.nan]),
        )
        res = dict(
            track_msgs=int(S["track_msgs"]),
            no_truth_for_stamp=int(S["no_truth_for_stamp"]),
            wrong_frame=int(S["wrong_frame"]),
            moving_coverage=round(S["moving_tracked"] / mf, 3),
            moving_coverage_after_1s=round(S["moving_tracked_settled"] / ms, 3),
            pos_rmse=round(float(np.sqrt(np.mean(pe**2))), 3),
            pos_p95=round(float(np.percentile(pe, 95)), 3),
            vel_rmse=round(float(np.sqrt(np.mean(ve**2))), 3),
            vel_p95=round(float(np.percentile(ve, 95)), 3),
            ids_per_person=round(
                float(np.mean([len(v) for v in self.ids.values()])) if self.ids else float("nan"),
                2,
            ),
            static_reported_frac=round(S["static_reported"] / max(S["static_frames"], 1), 3),
            ghosts_per_msg=round(S["ghost_tracks"] / max(S["track_msgs"], 1), 3),
            tracks_per_msg=round(S["tracks_published"] / max(S["track_msgs"], 1), 2),
            latency_ms_median=round(float(np.median(lat)) * 1000, 1),
            latency_ms_p95=round(float(np.percentile(lat, 95)) * 1000, 1),
        )
        json.dump(res, open(self.out, "w"), indent=1)
        print(json.dumps(res))
        raise SystemExit


def main():
    rclpy.init()
    n = Eval(sys.argv[1], float(sys.argv[2]))
    try:
        rclpy.spin(n)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
