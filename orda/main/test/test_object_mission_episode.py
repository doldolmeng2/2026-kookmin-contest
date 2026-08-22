import pytest

from main.object_mission_episode import ObjectMissionEpisodeGate


RELEASE_S = 1.90


def test_continuous_moving_episode_cannot_reenter_after_completion():
    gate = ObjectMissionEpisodeGate(RELEASE_S)
    gate.observe_valid_detection(1.0)
    assert gate.consume() is True
    for now in (1.5, 2.0, 2.8):
        gate.observe_valid_detection(now)
        assert gate.entry_allowed is False


def test_fixed_moving_class_flicker_does_not_create_a_new_episode():
    gate = ObjectMissionEpisodeGate(RELEASE_S)
    gate.observe_valid_detection(1.0)
    gate.consume()
    # The gate deliberately owns physical continuity, not the class label.
    gate.observe_valid_detection(1.7)
    gate.observe_valid_detection(2.4)
    assert gate.entry_allowed is False


def test_brief_gap_does_not_rearm():
    gate = ObjectMissionEpisodeGate(RELEASE_S)
    gate.observe_valid_detection(1.0)
    gate.consume()
    assert gate.expire(2.899) is False
    assert gate.entry_allowed is False


def test_sustained_clear_rearms_and_next_object_can_enter():
    gate = ObjectMissionEpisodeGate(RELEASE_S)
    gate.observe_valid_detection(1.0)
    gate.consume()
    assert gate.expire(2.90) is True
    assert gate.entry_allowed is False
    gate.observe_valid_detection(3.0)
    assert gate.entry_allowed is True


def test_bag_loop_reset_clears_all_episode_state():
    gate = ObjectMissionEpisodeGate(RELEASE_S)
    gate.observe_valid_detection(1.0)
    gate.consume()
    gate.reset()
    assert gate.episode_active is False
    assert gate.consumed is False
    assert gate.last_valid_detection_at is None


@pytest.mark.parametrize("value", [0.0, -1.0, float("inf")])
def test_release_duration_must_be_positive_finite(value):
    with pytest.raises(ValueError):
        ObjectMissionEpisodeGate(value)
