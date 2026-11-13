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

"""An obstacle is published after confirmation_frames frames inside the filters.

The velocity of a track is noisy, most of all right after its birth. With
position noise of a few centimetres a static object regularly shows more than
vel_filter[0] for a frame or two and would be published as a dynamic obstacle.

"""

from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
import pytest
import rclpy

from kf_hungarian_tracker.kf_hungarian_node import KFHungarianTracker

DT = 0.1


@pytest.fixture
def tracker():
    rclpy.init()
    node = KFHungarianTracker()
    # The node reads its parameters once, in __init__.
    node.transform_to_global_frame = False
    node.published = []
    node.tracker_obstacle_pub.get_subscription_count = lambda: 1
    node.tracker_obstacle_pub.publish = node.published.append
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _feed(node, frame, x):
    """Feed one detection at (x, 0) and return how many obstacles were published."""
    obstacle = Obstacle()
    obstacle.position.x = x
    msg = ObstacleArray()
    msg.header.frame_id = 'odom'
    t = 1.0 + frame * DT
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    msg.obstacles = [obstacle]
    node.callback(msg)
    return len(node.published[-1].obstacles)


def _first_published_frame(node, speed, frames=40):
    for frame in range(frames):
        if _feed(node, frame, speed * frame * DT) > 0:
            return frame
    return None


def test_default_publishes_as_soon_as_the_filters_pass(tracker):
    """confirmation_frames = 1 keeps the previous behaviour."""
    assert tracker.confirmation_frames == 1
    assert _first_published_frame(tracker, speed=1.0) is not None


@pytest.mark.parametrize('confirmation_frames', [3, 5])
def test_moving_obstacle_is_published_n_frames_later(confirmation_frames):
    """A steadily moving obstacle appears exactly confirmation_frames - 1 frames later."""
    firsts = []
    for n in (1, confirmation_frames):
        rclpy.init()
        node = KFHungarianTracker()
        node.transform_to_global_frame = False
        node.confirmation_frames = n
        node.published = []
        node.tracker_obstacle_pub.get_subscription_count = lambda: 1
        node.tracker_obstacle_pub.publish = node.published.append
        firsts.append(_first_published_frame(node, speed=1.0))
        node.destroy_node()
        rclpy.shutdown()
    assert firsts[0] is not None and firsts[1] is not None
    assert firsts[1] - firsts[0] == confirmation_frames - 1


@pytest.mark.parametrize('confirmation_frames, expect_published', [(1, True), (3, False)])
def test_one_noisy_frame_on_a_static_object(tracker, confirmation_frames, expect_published):
    """A static object jumps 20 cm for one frame: published at once, or not at all."""
    tracker.confirmation_frames = confirmation_frames
    # tight measurement noise, so that the jump shows as a clear velocity
    tracker.measurement_noise_cov = [0.01, 0.01, 0.01]
    published = 0
    for frame in range(30):
        published += _feed(tracker, frame, 0.2 if frame == 10 else 0.0)
    assert (published > 0) == expect_published, (
        f'confirmation_frames = {confirmation_frames}: a static object with one noisy '
        f'frame was published in {published} frames'
    )


def _feed_empty(node, frame):
    """Feed one frame with no detections and return how many were published."""
    msg = ObstacleArray()
    msg.header.frame_id = 'odom'
    t = 1.0 + frame * DT
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    node.callback(msg)
    return len(node.published[-1].obstacles)


def test_a_single_detection_is_not_published_while_the_track_coasts(tracker):
    """An obstacle seen once must not be confirmed by frames it was not seen on.

    The track outlives its only detection until death_threshold, and predict()
    keeps a velocity on it, so it keeps passing the filters.
    """
    tracker.confirmation_frames = 3
    # no velocity gating, so only the confirmation counter decides
    tracker.vel_filter = [-1.0, 10.0]
    published = _feed(tracker, 0, 0.0)
    published += sum(_feed_empty(tracker, frame) for frame in range(1, 20))
    assert published == 0, (
        f'confirmation_frames = 3: an obstacle detected on a single frame was '
        f'published in {published} frames '
    )


def test_a_missed_frame_pauses_the_count(tracker):
    """Detected, detected, missed, detected: published on the third detection."""
    tracker.confirmation_frames = 3
    tracker.vel_filter = [-1.0, 10.0]
    published = [
        _feed(tracker, 0, 0.0),
        _feed(tracker, 1, 0.0),
        _feed_empty(tracker, 2),
        _feed(tracker, 3, 0.0),
    ]
    assert published == [0, 0, 0, 1]


def test_a_confirmed_obstacle_stays_published_when_missed_once(tracker):
    """A missed frame does not unpublish an obstacle that is already confirmed."""
    tracker.confirmation_frames = 3
    tracker.vel_filter = [-1.0, 10.0]
    published = [_feed(tracker, frame, 0.0) for frame in range(5)]
    published.append(_feed_empty(tracker, 5))
    published.append(_feed(tracker, 6, 0.0))
    assert published == [0, 0, 1, 1, 1, 1, 1]
