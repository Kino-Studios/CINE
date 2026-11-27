import math

from cine_agent.setpoint_adjustment.utils.state_tracker import StateTracker


def test_depth_beyond_the_far_clip_is_not_smoothed_into_the_subject_depth():
    tracker = StateTracker()

    tracker.record_depth(4.0)
    tracker.record_depth(math.inf)
    tracker.record_depth(math.inf)

    assert tracker.get_smoothed_depth() == 4.0
