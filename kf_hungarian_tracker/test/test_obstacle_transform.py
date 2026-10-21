# Copyright 2026 Firefly Automatix, Inc.
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

"""An obstacle's box must survive the transform into global_frame.

Obstacle.msg describes a box by `position`, `orientation` and `size`, with
`size` measured along the box's own axes. Moving a box into another frame
moves its centre and turns its orientation; its size does not change.

"""

import math

from geometry_msgs.msg import TransformStamped
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
import numpy as np
import pytest
import rclpy
from scipy.spatial.transform import Rotation

from kf_hungarian_tracker.kf_hungarian_node import KFHungarianTracker

GLOBAL_FRAME = 'odom'
DETECTION_FRAME = 'base_footprint'

# A box 4 m long and 1 m wide along its own axes, 2 m tall, 5 m ahead.
BOX_SIZE = (4.0, 1.0, 2.0)
BOX_POSITION = (5.0, 0.0, 0.0)


@pytest.fixture
def tracker():
    rclpy.init()
    node = KFHungarianTracker()
    # The node reads its parameters once, in __init__.
    node.global_frame = GLOBAL_FRAME
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _yaw(q):
    """Heading of the quaternion's local x-axis."""
    forward = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([1.0, 0.0, 0.0])
    return math.atan2(forward[1], forward[0])


def _angle_between(a, b):
    """Smallest difference between two yaws, in degrees."""
    return abs(math.degrees((a - b + math.pi) % (2 * math.pi) - math.pi))


def _set_robot_yaw(node, yaw):
    """Robot (detection frame) at the global origin, turned `yaw` rad."""
    t = TransformStamped()
    t.header.frame_id = GLOBAL_FRAME
    t.child_frame_id = DETECTION_FRAME
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    node.tf_buffer.set_transform_static(t, 'test')


def _detect(node, box_yaw=0.0):
    """Feed one detection of the box and return the tracked obstacle."""
    obstacle = Obstacle()
    obstacle.position.x, obstacle.position.y, obstacle.position.z = BOX_POSITION
    obstacle.size.x, obstacle.size.y, obstacle.size.z = BOX_SIZE
    obstacle.orientation.z = math.sin(box_yaw / 2.0)
    obstacle.orientation.w = math.cos(box_yaw / 2.0)
    msg = ObstacleArray()
    msg.header.frame_id = DETECTION_FRAME
    msg.header.stamp.sec = 1
    msg.obstacles = [obstacle]

    node.callback(msg)

    assert len(node.obstacle_list) == 1, 'the detection did not start a track'
    return node.obstacle_list[0].msg


def _size(obstacle):
    return np.array([obstacle.size.x, obstacle.size.y, obstacle.size.z])


def test_orientation_defaults_to_identity():
    """A publisher that never sets orientation still means 'axis-aligned'."""
    assert Obstacle().orientation.w == 1.0


@pytest.mark.parametrize('robot_yaw_deg', [0.0, 30.0, 45.0, 90.0, -90.0, 135.0, 180.0])
def test_size_does_not_change_with_the_frame(tracker, robot_yaw_deg):
    """A 4 x 1 m box is 4 x 1 m in every frame; it is never negative."""
    _set_robot_yaw(tracker, math.radians(robot_yaw_deg))
    size = _size(_detect(tracker))
    np.testing.assert_allclose(
        size,
        BOX_SIZE,
        atol=1e-6,
        err_msg=(
            f'robot turned {robot_yaw_deg:+.0f} deg: a {BOX_SIZE[0]:g} x {BOX_SIZE[1]:g} m '
            f'box was tracked as {size[0]:.2f} x {size[1]:.2f} m. size is measured '
            'along the box, so moving it to another frame must not change it.'
        ),
    )


@pytest.mark.parametrize(
    'robot_yaw_deg, box_yaw_deg',
    [(0.0, 0.0), (45.0, 0.0), (90.0, 0.0), (0.0, -45.0), (30.0, 45.0), (-90.0, 135.0)],
)
def test_orientation_turns_with_the_frame(tracker, robot_yaw_deg, box_yaw_deg):
    """In global_frame the box is turned by the robot's yaw plus its own."""
    _set_robot_yaw(tracker, math.radians(robot_yaw_deg))
    tracked = _detect(tracker, math.radians(box_yaw_deg))
    expected = math.radians(robot_yaw_deg + box_yaw_deg)
    off = _angle_between(_yaw(tracked.orientation), expected)
    assert off < 1e-6, (
        f'robot turned {robot_yaw_deg:+.0f} deg, box turned {box_yaw_deg:+.0f} deg '
        f'from the robot: the track faces {math.degrees(_yaw(tracked.orientation)):+.1f} '
        f'deg in {GLOBAL_FRAME}, expected {robot_yaw_deg + box_yaw_deg:+.1f} deg.'
    )
