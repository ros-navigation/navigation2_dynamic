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

"""Detections must be moved into global_frame with the transform of their own stamp.

A detector needs time to produce its output, so a detection message arrives
later than the moment it describes. If the robot turned in between, the latest
transform puts every detection in the wrong place: something 5 m ahead of a
robot that turns at 0.6 rad/s is off by 0.3 m after 0.1 s. Static things then
seem to move and get tracked as dynamic obstacles.

"""

import math

from geometry_msgs.msg import TransformStamped
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
import pytest
import rclpy

from kf_hungarian_tracker.kf_hungarian_node import KFHungarianTracker

GLOBAL_FRAME = 'odom'
DETECTION_FRAME = 'base_footprint'

# The robot stands at the origin. It faces +x at t = 10 s and +y at t = 11 s.
T_SENSED = 10
T_LATEST = 11


@pytest.fixture
def tracker():
    rclpy.init()
    node = KFHungarianTracker()
    # The node reads its parameters once, in __init__.
    node.global_frame = GLOBAL_FRAME
    _set_robot_yaw(node, T_SENSED, 0.0)
    _set_robot_yaw(node, T_LATEST, math.pi / 2.0)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _set_robot_yaw(node, sec, yaw):
    t = TransformStamped()
    t.header.stamp.sec = sec
    t.header.frame_id = GLOBAL_FRAME
    t.child_frame_id = DETECTION_FRAME
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    node.tf_buffer.set_transform(t, 'test')


def _detect_5m_ahead(node, sec):
    """Feed one detection 5 m ahead of the robot, sensed at `sec`."""
    obstacle = Obstacle()
    obstacle.position.x = 5.0
    msg = ObstacleArray()
    msg.header.frame_id = DETECTION_FRAME
    msg.header.stamp.sec = sec
    msg.obstacles = [obstacle]

    node.callback(msg)

    assert len(node.obstacle_list) == 1, 'the detection did not start a track'
    return node.obstacle_list[0].msg.position


def test_detection_uses_the_transform_of_its_stamp(tracker):
    """Sensed while the robot faced +x: the obstacle is at (5, 0), not (0, 5)."""
    p = _detect_5m_ahead(tracker, T_SENSED)
    assert (p.x, p.y) == pytest.approx((5.0, 0.0), abs=1e-6), (
        f'a detection sensed at t = {T_SENSED} s, 5 m ahead of a robot that faced +x '
        f'at that time, was put at ({p.x:.2f}, {p.y:.2f}) in {GLOBAL_FRAME}. '
        f'(0, 5) is where the robot points at t = {T_LATEST} s, the latest transform.'
    )


def test_stamp_newer_than_the_buffer_falls_back_to_latest(tracker):
    """No transform for the stamp yet: use the latest one instead of dropping the message."""
    p = _detect_5m_ahead(tracker, T_LATEST + 5)
    assert (p.x, p.y) == pytest.approx((0.0, 5.0), abs=1e-6)
