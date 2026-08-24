import pytest

from bag_motion_replay.topic_sets import (
    JOYCON_TOPICS,
    TopicSelectionError,
    describe_selection,
    resolve_selection,
)

BAG_TOPICS = [
    '/camera_info',
    '/commands/motor/speed',
    '/commands/servo/position',
    '/image_raw',
    '/joy',
    '/parameter_events',
    '/point_cloud',
    '/resized_image',
    '/rosout',
    '/scan',
    '/sensors/servo_position_command',
    '/xycar_motor',
    '/xycar_ultrasonic',
]


def test_default_set_drives_the_actuation_layer_not_the_stick():
    selection = resolve_selection(BAG_TOPICS)

    assert selection.topics == (
        '/commands/servo/position',
        '/commands/motor/speed',
        '/sensors/servo_position_command',
    )
    assert '/joy' not in selection.topics
    assert '/xycar_motor' not in selection.topics


def test_mode_topic_absent_from_a_manual_bag_is_reported_not_fatal():
    selection = resolve_selection(BAG_TOPICS)

    assert '/mode_info' in selection.missing
    assert '/lane_info' in selection.missing
    assert '/mode_info' in describe_selection(selection)


def test_motor_set_is_empty_without_allow_joycon_and_says_why():
    # /xycar_motor is the joystick node's [steering, speed]; with it refused the
    # 'motor' set has nothing left in a manual bag, and the error has to explain
    # that rather than look like a missing topic.
    with pytest.raises(TopicSelectionError) as excinfo:
        resolve_selection(BAG_TOPICS, topic_set='motor')

    assert 'allow_joycon' in str(excinfo.value)
    assert '/xycar_motor' in str(excinfo.value)


def test_allow_joycon_opts_back_in():
    selection = resolve_selection(BAG_TOPICS, topic_set='motor', allow_joycon=True)

    assert '/xycar_motor' in selection.topics
    assert selection.refused_joycon == ()


def test_naming_a_joycon_topic_explicitly_is_an_error_not_a_silent_drop():
    with pytest.raises(TopicSelectionError) as excinfo:
        resolve_selection(BAG_TOPICS, include_topics=['/joy'])

    assert 'allow_joycon' in str(excinfo.value)


def test_every_joycon_topic_is_covered_by_the_refusal():
    for topic in JOYCON_TOPICS:
        with pytest.raises(TopicSelectionError):
            resolve_selection(BAG_TOPICS, include_topics=[topic])


def test_exclude_wins_over_the_set():
    selection = resolve_selection(
        BAG_TOPICS, exclude_topics=['/sensors/servo_position_command']
    )

    assert '/sensors/servo_position_command' not in selection.topics
    assert any('excluded by request' in note for note in selection.notes)


def test_all_set_skips_infrastructure_topics():
    selection = resolve_selection(BAG_TOPICS, topic_set='all')

    assert '/rosout' not in selection.topics
    assert '/parameter_events' not in selection.topics
    assert '/scan' in selection.topics


def test_unknown_set_is_rejected_with_the_known_ones_listed():
    with pytest.raises(TopicSelectionError) as excinfo:
        resolve_selection(BAG_TOPICS, topic_set='nonsense')

    assert 'actuation' in str(excinfo.value)


def test_explicit_topic_missing_from_the_bag_is_an_error():
    with pytest.raises(TopicSelectionError):
        resolve_selection(BAG_TOPICS, include_topics=['/nope'])


def test_empty_result_is_an_error_rather_than_a_silent_no_op():
    with pytest.raises(TopicSelectionError):
        resolve_selection(['/rosout'], topic_set='actuation')


def test_include_can_add_a_topic_outside_any_set():
    selection = resolve_selection(BAG_TOPICS, include_topics=['/scan'])

    assert '/scan' in selection.topics
