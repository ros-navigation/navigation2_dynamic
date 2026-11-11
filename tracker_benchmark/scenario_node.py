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

"""Ground-truth scenario generator for evaluating a multi-object tracker (nav2_dynamic_msgs).

A robot moves in the odom frame (profiles: static / straight / spin / arc). People walk on scripted
paths, some stop for a while; static boxes stand around. A simulated "detector" publishes
ObstacleArray detections in the robot frame (base_link) with position noise, missed detections,
false positives, limited range and a processing delay. Ground truth is published separately.

  /detection  nav2_dynamic_msgs/ObstacleArray  frame base_link, stamp = sensing time, velocity = 0
  /truth      nav2_dynamic_msgs/ObstacleArray  frame odom, true position / velocity, fixed uuid
  /tf         odom -> base_link
Parameters via environment (so runs are scriptable): see CFG below.
"""

import math
import os
import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from tf2_ros import TransformBroadcaster
from unique_identifier_msgs.msg import UUID

CFG = dict(
    profile=os.environ.get("B_PROFILE", "straight"),  # static | straight | spin | arc
    seed=int(os.environ.get("B_SEED", "1")),
    duration=float(os.environ.get("B_DURATION", "60")),
    rate=float(os.environ.get("B_RATE", "10")),  # detector rate, Hz
    noise=float(os.environ.get("B_NOISE", "0.05")),  # position noise sigma, m
    p_miss=float(os.environ.get("B_PMISS", "0.1")),  # probability a visible object is not detected
    fp_rate=float(os.environ.get("B_FP", "0.2")),  # false positives per frame (Poisson mean)
    det_range=float(os.environ.get("B_RANGE", "8.0")),  # m
    delay=float(os.environ.get("B_DELAY", "0.1")),  # detector processing delay, s
    n_people=int(os.environ.get("B_PEOPLE", "6")),
    n_static=int(os.environ.get("B_STATIC", "6")),
)


class Walker:
    """Walks between random waypoints inside a box around the robot path; sometimes pauses."""

    def __init__(self, rng, idx):
        self.rng, self.idx = rng, idx
        self.speed = float(rng.uniform(0.5, 1.6))
        self.p = np.array([rng.uniform(-4, 12), rng.uniform(-5, 5)])
        self.v = np.zeros(2)
        self.pause = 0.0
        self.new_goal()

    def new_goal(self):
        self.goal = np.array([self.rng.uniform(-4, 12), self.rng.uniform(-5, 5)])

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
            self.new_goal()
            return
        self.v = d / n * self.speed
        self.p = self.p + self.v * dt


def uuid_of(i):
    u = UUID()
    u.uuid = [0] * 15 + [i + 1]
    return u


class Scenario(Node):
    def __init__(self):
        super().__init__("scenario")
        c = CFG
        self.rng = np.random.default_rng(c["seed"])
        self.people = [Walker(self.rng, i) for i in range(c["n_people"])]
        self.static = [
            np.array([self.rng.uniform(-4, 12), self.rng.uniform(-5, 5)])
            for _ in range(c["n_static"])
        ]
        self.pose = np.zeros(3)  # x, y, yaw in odom
        self.tfb = TransformBroadcaster(self)
        self.pub_det = self.create_publisher(ObstacleArray, "detection", 10)
        self.pub_truth = self.create_publisher(ObstacleArray, "truth", 10)
        self.pending = []  # (publish_time, msg)
        self.t0 = time.monotonic()
        self.t_prev = 0.0
        self.next_det = 1.0
        self.create_timer(0.02, self.tick)

    def robot_cmd(self, t):
        p = CFG["profile"]
        if p == "static":
            return 0.0, 0.0
        if p == "straight":
            return 0.5, 0.0
        if p == "spin":
            return 0.0, 0.6 * (1 if int(t / 8) % 2 == 0 else -1)
        return 0.5, 0.3 * math.sin(t / 4.0)  # arc

    def tick(self):
        t = time.monotonic() - self.t0
        dt = min(t - self.t_prev, 0.1)
        self.t_prev = t
        v, w = self.robot_cmd(t)
        self.pose[0] += v * math.cos(self.pose[2]) * dt
        self.pose[1] += v * math.sin(self.pose[2]) * dt
        self.pose[2] += w * dt
        for p in self.people:
            p.step(dt)
        now = self.get_clock().now()
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y = float(self.pose[0]), float(
            self.pose[1]
        )
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.pose[2] / 2), math.cos(
            self.pose[2] / 2
        )
        self.tfb.sendTransform(tf)
        while self.pending and self.pending[0][0] <= t:
            self.pub_det.publish(self.pending.pop(0)[1])
        if t >= self.next_det:
            self.next_det += 1.0 / CFG["rate"]
            self.sense(t, now)
        if t > CFG["duration"]:
            raise SystemExit

    def sense(self, t, now):
        c = CFG
        stamp = now.to_msg()
        truth = ObstacleArray()
        truth.header.stamp = stamp
        truth.header.frame_id = "odom"
        det = ObstacleArray()
        det.header.stamp = stamp
        det.header.frame_id = "base_link"
        cy, sy = math.cos(self.pose[2]), math.sin(self.pose[2])
        objs = [(i, p.p, p.v, (0.5, 0.5, 1.7)) for i, p in enumerate(self.people)]
        objs += [(100 + i, s, np.zeros(2), (0.6, 0.6, 1.0)) for i, s in enumerate(self.static)]
        for oid, p, v, size in objs:
            o = Obstacle()
            o.uuid = uuid_of(oid)
            o.position.x, o.position.y, o.position.z = float(p[0]), float(p[1]), size[2] / 2
            o.velocity.x, o.velocity.y = float(v[0]), float(v[1])
            o.size.x, o.size.y, o.size.z = size
            o.orientation.w = 1.0
            rel = p - self.pose[:2]
            o.score = float(
                np.linalg.norm(rel) <= c["det_range"]
            )  # score 1 = inside detector range
            truth.obstacles.append(o)
            if o.score < 0.5 or self.rng.random() < c["p_miss"]:
                continue
            d = Obstacle()
            d.score = 0.9
            n = self.rng.normal(0, c["noise"], 2)
            d.position.x = float(cy * rel[0] + sy * rel[1] + n[0])
            d.position.y = float(-sy * rel[0] + cy * rel[1] + n[1])
            d.position.z = size[2] / 2
            d.size.x, d.size.y, d.size.z = size
            d.orientation.w = 1.0
            det.obstacles.append(d)
        for _ in range(self.rng.poisson(c["fp_rate"])):
            d = Obstacle()
            d.score = 0.3
            a = self.rng.uniform(0, 2 * math.pi)
            r = self.rng.uniform(1, c["det_range"])
            d.position.x, d.position.y, d.position.z = (
                float(r * math.cos(a)),
                float(r * math.sin(a)),
                0.5,
            )
            d.size.x = d.size.y = d.size.z = 0.4
            d.orientation.w = 1.0
            det.obstacles.append(d)
        self.pub_truth.publish(truth)
        self.pending.append((t + c["delay"], det))


def main():
    rclpy.init()
    n = Scenario()
    n.get_logger().info("scenario " + json.dumps(CFG))
    try:
        rclpy.spin(n)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
