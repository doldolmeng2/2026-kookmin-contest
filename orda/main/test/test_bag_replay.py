import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from main import bag_replay
from main import cone_entry, control_selector, race_fsm
from main.bag_replay import (
    BagEvent,
    OfflineBagReplay,
    parse_cone_message,
    write_report_files,
)
from main.cone_entry import ConeEntryConfig
from main.race_fsm import Mode


TYPE_BY_TOPIC = {
    "/traffic_detection": "std_msgs/msg/Bool",
    "/rubbercone_info": "std_msgs/msg/Int32MultiArray",
    "/lane_offset": "std_msgs/msg/Int16",
    "/mode_info": "std_msgs/msg/Int32MultiArray",
    "/xycar_motor": "std_msgs/msg/Float32MultiArray",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/lane_valid": "std_msgs/msg/Bool",
}


def event(topic, timestamp_s, data=None):
    message = SimpleNamespace(data=data) if topic != "/scan" else SimpleNamespace()
    return BagEvent(
        topic=topic,
        type_name=TYPE_BY_TOPIC[topic],
        timestamp_ns=int(timestamp_s * 1_000_000_000),
        message=message,
    )


def replay(
    events,
    start_mode=Mode.INIT,
    topic_types=None,
    cone_entry_config=None,
):
    if topic_types is None:
        topic_types = {
            item.topic: item.type_name
            for item in events
        }
    return OfflineBagReplay(
        start_mode=start_mode,
        cone_entry_config=cone_entry_config,
    ).run(
        events,
        bag_path=Path("/tmp/unit_test_bag"),
        topic_types=topic_types,
    )


def test_bag_timestamp_is_the_observation_and_transition_clock():
    report = replay(
        [
            event("/traffic_detection", 100.0, False),
            event("/traffic_detection", 101.0, True),
            event("/traffic_detection", 102.0, True),
            event("/traffic_detection", 103.0, True),
        ],
        start_mode=Mode.WAIT_GREEN,
    )

    assert report["fsm"]["final_mode"] == "LANE_DRIVE"
    assert report["fsm"]["state_entered_at_s"] == 103.0
    assert report["fsm"]["race_started_at_s"] == 101.0
    assert report["fsm"]["transition_timeline"] == [
        {
            "timestamp_s": 103.0,
            "relative_time_s": 3.0,
            "source_mode": "WAIT_GREEN",
            "target_mode": "LANE_DRIVE",
            "reason": "green signal debounced",
        }
    ]


def test_replay_module_does_not_use_a_wall_clock():
    source = inspect.getsource(bag_replay)
    assert "time.time(" not in source
    assert "time.monotonic(" not in source


def test_records_are_processed_in_timestamp_order_from_reader_sequence():
    report = replay(
        [
            event("/traffic_detection", 1.0, False),
            event("/traffic_detection", 2.0, True),
            event("/traffic_detection", 3.0, False),
        ],
        start_mode=Mode.WAIT_GREEN,
    )

    assert [
        entry["timestamp_s"] for entry in report["traffic"]["timeline"]
    ] == [1.0, 2.0, 3.0]
    assert report["validation"]["invariants"][
        "accepted_timestamps_monotonic"
    ] is True


def test_same_topic_timestamp_is_counted_once_as_an_event_edge():
    report = replay(
        [
            event("/traffic_detection", 1.0, True),
            event("/traffic_detection", 1.0, True),
            event("/traffic_detection", 2.0, True),
            event("/traffic_detection", 3.0, True),
        ],
        start_mode=Mode.WAIT_GREEN,
    )

    assert report["processing"]["duplicate_record_count"] == 1
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"][0]["timestamp_s"] == 3.0
    assert len(report["traffic"]["timeline"]) == 3


def test_missing_topics_are_warnings_instead_of_replay_failure():
    report = replay(
        [event("/traffic_detection", 1.0, False)],
        start_mode=Mode.WAIT_GREEN,
        topic_types={"/traffic_detection": TYPE_BY_TOPIC["/traffic_detection"]},
    )

    assert "/rubbercone_info" in report["basic_info"]["missing_topics"]
    assert any(
        warning == "missing topic: /rubbercone_info"
        for warning in report["validation"]["warnings"]
    )


def test_three_field_and_two_field_cone_messages_are_distinct():
    three_fields = parse_cone_message(SimpleNamespace(data=[-42, 0, 97]))
    two_fields = parse_cone_message(SimpleNamespace(data=[-11, 1]))

    assert three_fields == {
        "offset": -42,
        "end_flag": 0,
        "confidence": 97,
        "field_count": 3,
        "malformed": False,
    }
    assert two_fields["offset"] == -11
    assert two_fields["end_flag"] == 1
    assert two_fields["confidence"] is None
    assert two_fields["malformed"] is False


def test_malformed_cone_message_is_warned_and_not_mission_evidence():
    report = replay(
        [event("/rubbercone_info", 1.0, [7])],
        start_mode=Mode.LANE_DRIVE,
    )

    entry = report["rubbercone"]["timeline"][0]
    assert entry["malformed"] is True
    assert entry["candidate_evidence"] is False
    assert report["rubbercone"]["statistics"]["malformed_message_count"] == 1
    assert any(
        "malformed rubbercone message" in warning
        for warning in report["validation"]["warnings"]
    )


def test_duplicate_cone_record_is_not_reused_as_a_new_event():
    report = replay(
        [
            event("/rubbercone_info", 1.0, [5, 0, 80]),
            event("/rubbercone_info", 1.0, [5, 0, 80]),
            event("/scan", 2.0),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert len(report["rubbercone"]["timeline"]) == 1
    assert report["rubbercone"]["statistics"][
        "same_timestamp_duplicate_count"
    ] == 1
    assert report["fsm"]["final_mode"] == "LANE_DRIVE"


def test_replay_enters_cone_drive_on_unique_cone_edges_only():
    report = replay(
        [
            event("/scan", 0.99),
            event("/rubbercone_info", 1.0, [10, 0, 90]),
            event("/lane_offset", 1.02, 4),
            event("/scan", 1.09),
            event("/rubbercone_info", 1.1, [12, 0, 91]),
            event("/mode_info", 1.15, [1, 1]),
            event("/xycar_motor", 1.16, [2.0, 0.1]),
            event("/traffic_detection", 1.17, False),
            event("/scan", 1.2),
            event("/rubbercone_info", 1.21, [14, 0, 92]),
            event("/rubbercone_info", 1.31, [15, 0, 93]),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert report["fsm"]["final_mode"] == "CONE_DRIVE"
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["cone_entered_at_s"] == 1.21
    assert report["fsm"]["transition_timeline"] == [
        {
            "timestamp_s": 1.21,
            "relative_time_s": 0.22,
            "source_mode": "LANE_DRIVE",
            "target_mode": "CONE_DRIVE",
            "reason": "cone entry confirmed",
        }
    ]
    evaluated = [
        item
        for item in report["rubbercone"]["timeline"]
        if item["guard_evaluated"]
    ]
    assert [item["guard_qualifying_message_count"] for item in evaluated] == [
        1,
        2,
        3,
    ]


def test_replay_enters_rejoin_once_on_new_zero_to_one_session():
    report = replay(
        [
            event("/scan", 0.9),
            event("/rubbercone_info", 1.0, [4, 1, 0]),
            event("/rubbercone_info", 1.1, [4, 1, 0]),
            event("/rubbercone_info", 1.2, [4, 0, 80]),
            event("/rubbercone_info", 1.3, [4, 1, 0]),
            event("/rubbercone_info", 1.4, [4, 1, 0]),
        ],
        start_mode=Mode.CONE_DRIVE,
    )

    assert report["fsm"]["final_mode"] == "REJOIN"
    assert report["fsm"]["state_entered_at_s"] == 1.3
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"] == [
        {
            "timestamp_s": 1.3,
            "relative_time_s": 0.4,
            "source_mode": "CONE_DRIVE",
            "target_mode": "REJOIN",
            "reason": "fresh cone end flag",
        }
    ]
    assert report["validation"]["invariants"][
        "only_allowed_transitions_observed"
    ] is True


def test_replay_latched_one_sequence_never_ends_unarmed_session():
    report = replay(
        [
            event("/scan", 0.9),
            event("/rubbercone_info", 1.0, [0, 1, 0]),
            event("/scan", 1.1),
            event("/rubbercone_info", 1.2, [0, 1, 0]),
            event("/rubbercone_info", 1.3, [0, 1, 0]),
        ],
        start_mode=Mode.CONE_DRIVE,
    )

    assert report["fsm"]["final_mode"] == "CONE_DRIVE"
    assert report["fsm"]["transition_count"] == 0
    assert report["fsm"]["state_entered_at_s"] == 0.9


def test_replay_rejects_regressed_end_before_accepting_newer_end():
    report = replay(
        [
            event("/scan", 1.8),
            event("/rubbercone_info", 2.0, [0, 0, 80]),
            event("/rubbercone_info", 1.9, [0, 1, 0]),
            event("/scan", 2.1),
            event("/rubbercone_info", 2.2, [0, 1, 0]),
        ],
        start_mode=Mode.CONE_DRIVE,
    )

    assert report["processing"]["timestamp_regression_count"] == 1
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"][0]["timestamp_s"] == 2.2


@pytest.mark.parametrize("shift", [0.0, 60.0, 3600.0])
def test_replay_cone_exit_depends_on_relative_freshness_not_absolute_time(shift):
    report = replay(
        [
            event("/scan", shift + 0.9),
            event("/rubbercone_info", shift + 1.0, [0, 0, 80]),
            event("/rubbercone_info", shift + 1.1, [0, 1, 0]),
        ],
        start_mode=Mode.CONE_DRIVE,
    )

    assert report["fsm"]["final_mode"] == "REJOIN"
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"][0][
        "relative_time_s"
    ] == pytest.approx(0.2, abs=TIMING_TOLERANCE_S)


def test_duplicate_cone_edge_does_not_advance_replay_guard():
    report = replay(
        [
            event("/scan", 0.9),
            event("/rubbercone_info", 1.0, [0, 0, 90]),
            event("/rubbercone_info", 1.0, [0, 0, 90]),
            event("/scan", 1.09),
            event("/rubbercone_info", 1.1, [0, 0, 90]),
            event("/scan", 1.2),
            event("/rubbercone_info", 1.21, [0, 0, 90]),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert report["processing"]["duplicate_record_count"] == 1
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"][0]["timestamp_s"] == 1.21


def test_two_field_cone_messages_keep_confidence_none_and_never_enter():
    events = []
    for timestamp in (1.0, 1.1, 1.3, 1.5):
        events.extend(
            [
                event("/scan", timestamp - 0.01),
                event("/rubbercone_info", timestamp, [0, 0]),
            ]
        )

    report = replay(events, start_mode=Mode.LANE_DRIVE)

    assert report["fsm"]["final_mode"] == "LANE_DRIVE"
    assert report["fsm"]["transition_count"] == 0
    assert all(
        item["confidence"] is None
        for item in report["rubbercone"]["timeline"]
    )
    assert all(
        item["guard_reason"] == "cone confidence missing"
        for item in report["rubbercone"]["timeline"]
    )


def test_synthetic_negative_replay_does_not_enter_cone_drive():
    events = []
    samples = [
        (1.0, 100, 0),
        (1.1, 20, 0),
        (1.2, 74, 0),
        (1.3, 74, 0),
        (1.4, 95, 0),
        (1.5, 96, 1),
    ]
    for timestamp, confidence, end_flag in samples:
        events.extend(
            [
                event("/scan", timestamp - 0.01),
                event(
                    "/rubbercone_info",
                    timestamp,
                    [0, end_flag, confidence],
                ),
            ]
        )

    report = replay(events, start_mode=Mode.LANE_DRIVE)

    assert report["fsm"]["final_mode"] == "LANE_DRIVE"
    assert report["fsm"]["transition_count"] == 0


def test_replay_report_records_provisional_cone_config():
    config = ConeEntryConfig(
        min_confidence=80,
        min_messages=4,
        min_duration_s=0.3,
        max_cone_age_s=0.2,
        max_scan_age_s=0.2,
    )
    report = replay(
        [event("/scan", 1.0)],
        start_mode=Mode.LANE_DRIVE,
        cone_entry_config=config,
    )

    assert report["cone_entry_guard"]["config"] == {
        "min_confidence": 80,
        "min_messages": 4,
        "min_duration_s": 0.3,
        "max_cone_age_s": 0.2,
        "max_scan_age_s": 0.2,
    }
    assert report["cone_entry_guard"]["default_is_provisional"] is True
    assert report["cone_entry_guard"]["using_default_config"] is False
    assert "not a final threshold" in report["cone_entry_guard"]["warning"]


def test_lane_offset_is_reference_only_and_never_infers_lane_validity():
    report = replay(
        [
            event("/lane_offset", 1.0, 12),
            event("/lane_offset", 2.0, 12),
        ],
        start_mode=Mode.REJOIN,
    )

    assert report["fsm"]["final_mode"] == "REJOIN"
    assert report["fsm"]["transition_count"] == 0
    assert report["fsm"]["fsm_evaluation_count"] == 0
    assert report["lane_reference"]["lane_validity_inferred"] is False


def test_replay_rejoin_returns_to_lane_only_on_fresh_lane_validity_edges():
    report = replay(
        [
            event("/scan", 1.0),
            event("/lane_valid", 1.1, True),
            event("/lane_valid", 1.2, True),
            event("/lane_valid", 1.31, True),
        ],
        start_mode=Mode.REJOIN,
    )

    assert report["fsm"]["final_mode"] == "LANE_DRIVE"
    assert report["fsm"]["state_entered_at_s"] == 1.31
    assert report["fsm"]["transition_timeline"] == [
        {
            "timestamp_s": 1.31,
            "relative_time_s": 0.31,
            "source_mode": "REJOIN",
            "target_mode": "LANE_DRIVE",
            "reason": "fresh lane validity confirmed",
        }
    ]
    assert report["validation"]["invariants"][
        "only_allowed_transitions_observed"
    ] is True
    assert any(
        "REJOIN to LANE_DRIVE" in item
        for item in report["validation"]["verifiable_scope"]
    )


def test_legacy_mode_trace_never_changes_the_new_mode():
    report = replay(
        [
            event("/mode_info", 1.0, [1, 1]),
            event("/mode_info", 2.0, [4, 2]),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert report["fsm"]["final_mode"] == "LANE_DRIVE"
    assert report["legacy_reference"]["used_as_new_mode_label"] is False
    assert len(report["legacy_reference"]["legacy_mode_changes"]) == 2


def test_recorded_motor_values_only_produce_reference_statistics():
    report = replay(
        [
            event("/xycar_motor", 1.0, [-5.0, 0.1]),
            event("/xycar_motor", 2.0, [7.0, 0.2]),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    motor = report["recorded_motor_reference"]
    assert motor["recorded_message_count"] == 2
    assert motor["angle"]["min"] == -5.0
    assert motor["angle"]["max"] == 7.0
    assert motor["speed"]["max"] == 0.2
    assert motor["ros_output_attempted"] is False
    assert report["fsm"]["transition_count"] == 0


def test_only_real_mode_changes_create_transition_logs():
    report = replay(
        [
            event("/traffic_detection", 1.0, False),
            event("/traffic_detection", 2.0, True),
            event("/traffic_detection", 3.0, True),
            event("/traffic_detection", 4.0, True),
            event("/traffic_detection", 5.0, True),
            event("/scan", 6.0),
        ],
        start_mode=Mode.WAIT_GREEN,
    )

    assert report["fsm"]["fsm_evaluation_count"] == 6
    assert report["fsm"]["transition_count"] == 1
    assert report["validation"]["invariants"][
        "at_most_one_transition_log_per_event"
    ] is True


def _terminal_probe_events():
    return [
        event("/traffic_detection", 1.0, True),
        event("/rubbercone_info", 2.0, [0, 1, 0]),
        event("/scan", 3.0),
    ]


def test_finish_is_sticky():
    report = replay(_terminal_probe_events(), start_mode=Mode.FINISH)

    assert report["fsm"]["final_mode"] == Mode.FINISH.value
    assert report["fsm"]["transition_count"] == 0
    assert report["validation"]["invariants"]["terminal_modes_are_sticky"] is True


def test_stop_recovers_once_inputs_are_healthy_again():
    """STOP은 종료 상태가 아니라 복귀 가능한 안전 정지다.

    센서가 잠깐 끊겼다고 주행이 영영 끝나면 안 된다. 규정상 정지 후 1분 내
    미재개는 실격이다.
    """
    report = replay(_terminal_probe_events(), start_mode=Mode.STOP)

    assert report["fsm"]["final_mode"] != Mode.STOP.value
    assert report["fsm"]["transition_count"] >= 1


def test_timestamp_regression_is_rejected_and_reported():
    report = replay(
        [
            event("/traffic_detection", 2.0, True),
            event("/traffic_detection", 1.0, True),
            event("/traffic_detection", 3.0, True),
            event("/traffic_detection", 4.0, True),
        ],
        start_mode=Mode.WAIT_GREEN,
    )

    assert report["processing"]["timestamp_regression_count"] == 1
    assert len(report["traffic"]["timeline"]) == 3
    assert any(
        "timestamp regression rejected" in warning
        for warning in report["validation"]["warnings"]
    )
    assert report["validation"]["invariants"][
        "accepted_timestamps_monotonic"
    ] is False


def test_regressed_cone_event_resets_replay_guard_before_later_frames():
    report = replay(
        [
            event("/scan", 0.99),
            event("/rubbercone_info", 1.0, [0, 0, 90]),
            event("/scan", 1.09),
            event("/rubbercone_info", 1.1, [0, 0, 90]),
            event("/rubbercone_info", 1.05, [0, 0, 90]),
            event("/scan", 1.19),
            event("/rubbercone_info", 1.2, [0, 0, 90]),
            event("/scan", 1.29),
            event("/rubbercone_info", 1.3, [0, 0, 90]),
            event("/scan", 1.5),
            event("/rubbercone_info", 1.51, [0, 0, 90]),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert report["processing"]["timestamp_regression_count"] == 1
    assert report["fsm"]["transition_count"] == 1
    assert report["fsm"]["transition_timeline"][0]["timestamp_s"] == 1.51


def test_self_transition_keeps_initial_state_entry_timestamp():
    report = replay(
        [
            event("/rubbercone_info", 10.0, [1, 0, 20]),
            event("/rubbercone_info", 11.0, [2, 0, 30]),
            event("/scan", 12.0),
        ],
        start_mode=Mode.LANE_DRIVE,
    )

    assert report["fsm"]["state_entered_at_s"] == 10.0
    assert report["fsm"]["transition_count"] == 0
    assert report["validation"]["invariants"][
        "self_transition_preserves_state_entered_at"
    ] is True


def test_required_json_schema_and_report_files(tmp_path):
    report = replay(
        [
            event("/traffic_detection", 1.0, False),
            event("/rubbercone_info", 2.0, [-3, 0, 50]),
            event("/lane_offset", 3.0, 4),
            event("/mode_info", 4.0, [1, 1]),
            event("/xycar_motor", 5.0, [2.0, 0.1]),
            event("/scan", 6.0),
        ],
        start_mode=Mode.LANE_DRIVE,
    )
    required = {
        "basic_info",
        "fsm",
        "cone_entry_guard",
        "traffic",
        "rubbercone",
        "lane_reference",
        "legacy_reference",
        "recorded_motor_reference",
        "scan_reference",
        "processing",
        "validation",
    }
    assert required <= set(report)

    json_path, text_path = write_report_files(report, tmp_path)
    saved = json.loads(json_path.read_text(encoding="utf-8"))

    assert json_path.name == "unit_test_bag.json"
    assert text_path.name == "unit_test_bag.txt"
    assert json_path.parent.name == "LANE_DRIVE"
    assert required <= set(saved)
    assert "Offline FSM bag replay report" in text_path.read_text(encoding="utf-8")


TIMING_TOLERANCE_S = 1e-6


def qualifying_cone_events(shift=0.0, interleaved=False):
    events = [
        event("/scan", shift + 0.99),
        event("/rubbercone_info", shift + 1.0, [10, 0, 90]),
    ]
    if interleaved:
        events.extend(
            [
                event("/lane_offset", shift + 1.01, 7),
                event("/mode_info", shift + 1.02, [9, 9]),
                event("/xycar_motor", shift + 1.03, [20.0, 10.0]),
                event("/traffic_detection", shift + 1.04, False),
                event("/scan", shift + 1.05),
            ]
        )
    events.extend(
        [
            event("/scan", shift + 1.09),
            event("/rubbercone_info", shift + 1.1, [11, 0, 91]),
        ]
    )
    if interleaved:
        events.extend(
            [
                event("/lane_offset", shift + 1.11, -4),
                event("/mode_info", shift + 1.12, [1, 2]),
                event("/xycar_motor", shift + 1.13, [-10.0, 8.0]),
                event("/traffic_detection", shift + 1.14, False),
                event("/scan", shift + 1.15),
            ]
        )
    events.extend(
        [
            event("/scan", shift + 1.2),
            event("/rubbercone_info", shift + 1.21, [12, 0, 92]),
        ]
    )
    return events


def test_replay_timestamp_shift_changes_only_absolute_cone_times():
    baseline = replay(
        qualifying_cone_events(),
        start_mode=Mode.LANE_DRIVE,
    )
    baseline_diagnostics = baseline["cone_entry_guard"]["diagnostics"]

    for shift in (5.0, 20.0, 60.0, 3600.0):
        shifted = replay(
            qualifying_cone_events(shift),
            start_mode=Mode.LANE_DRIVE,
        )
        diagnostics = shifted["cone_entry_guard"]["diagnostics"]

        assert shifted["fsm"]["final_mode"] == baseline["fsm"]["final_mode"]
        assert shifted["fsm"]["transition_count"] == 1
        assert shifted["fsm"]["transition_timeline"][0]["reason"] == (
            "cone entry confirmed"
        )
        assert diagnostics["qualifying_unique_messages"] == 3
        assert diagnostics["trigger_delay_from_first_qualifying_s"] == (
            pytest.approx(
                baseline_diagnostics[
                    "trigger_delay_from_first_qualifying_s"
                ],
                abs=TIMING_TOLERANCE_S,
            )
        )
        assert diagnostics["transition_relative_time_s"] == pytest.approx(
            baseline_diagnostics["transition_relative_time_s"],
            abs=TIMING_TOLERANCE_S,
        )
        assert diagnostics["transition_time_s"] - baseline_diagnostics[
            "transition_time_s"
        ] == pytest.approx(shift, abs=TIMING_TOLERANCE_S)
        assert diagnostics["first_qualifying_time_s"] - baseline_diagnostics[
            "first_qualifying_time_s"
        ] == pytest.approx(shift, abs=TIMING_TOLERANCE_S)


@pytest.mark.parametrize("prefix_duration_s", [5.0, 20.0, 60.0])
def test_lane_prefix_does_not_change_delay_from_qualifying_onset(
    prefix_duration_s,
):
    qualifying_start = prefix_duration_s + 1.0
    events = [
        event("/scan", 0.0),
        event("/lane_offset", 0.1, 1),
        event("/mode_info", 0.2, [1, 1]),
        event("/xycar_motor", 0.3, [0.0, 3.0]),
        event("/traffic_detection", 0.4, False),
        event("/scan", prefix_duration_s - 0.5),
        event(
            "/rubbercone_info",
            prefix_duration_s - 0.49,
            [0, 0, 74],
        ),
        event("/traffic_detection", prefix_duration_s - 0.3, False),
        event("/lane_offset", prefix_duration_s - 0.2, -1),
        event("/scan", qualifying_start - 0.01),
        event("/rubbercone_info", qualifying_start, [10, 0, 90]),
        event("/scan", qualifying_start + 0.09),
        event("/rubbercone_info", qualifying_start + 0.1, [11, 0, 91]),
        event("/scan", qualifying_start + 0.2),
        event("/rubbercone_info", qualifying_start + 0.21, [12, 0, 92]),
    ]

    report = replay(events, start_mode=Mode.LANE_DRIVE)
    diagnostics = report["cone_entry_guard"]["diagnostics"]

    assert report["fsm"]["final_mode"] == "CONE_DRIVE"
    assert report["fsm"]["transition_count"] == 1
    assert diagnostics["qualifying_unique_messages"] == 3
    assert diagnostics["trigger_delay_from_first_qualifying_s"] == (
        pytest.approx(0.21, abs=TIMING_TOLERANCE_S)
    )
    assert diagnostics["first_qualifying_relative_time_s"] == (
        pytest.approx(qualifying_start, abs=TIMING_TOLERANCE_S)
    )


def test_irrelevant_topic_interleave_does_not_count_cached_cone_frames():
    baseline = replay(
        qualifying_cone_events(),
        start_mode=Mode.LANE_DRIVE,
    )
    interleaved = replay(
        qualifying_cone_events(interleaved=True),
        start_mode=Mode.LANE_DRIVE,
    )

    baseline_diagnostics = baseline["cone_entry_guard"]["diagnostics"]
    interleaved_diagnostics = interleaved["cone_entry_guard"]["diagnostics"]
    assert interleaved_diagnostics["qualifying_unique_messages"] == (
        baseline_diagnostics["qualifying_unique_messages"]
    ) == 3
    assert interleaved_diagnostics[
        "trigger_delay_from_first_qualifying_s"
    ] == pytest.approx(
        baseline_diagnostics["trigger_delay_from_first_qualifying_s"],
        abs=TIMING_TOLERANCE_S,
    )
    assert interleaved["fsm"]["transition_timeline"] == baseline["fsm"][
        "transition_timeline"
    ]


def test_runtime_guard_and_selector_have_no_bag_identity_or_expected_time():
    runtime_source = "\n".join(
        inspect.getsource(module)
        for module in (cone_entry, race_fsm, control_selector)
    )
    forbidden_identities = (
        "bag_path",
        "bag_name",
        "reverse_20260725_222239",
        "rubbercone_20260725_221245",
        "tuned_forward",
        "tuned_retry",
        "5.248707",
        "8.208243",
        "5.844619",
        "4.910223",
    )

    assert all(identity not in runtime_source for identity in forbidden_identities)


def test_report_marks_cone_timing_as_relative_diagnostic_not_ground_truth():
    report = replay(
        qualifying_cone_events(),
        start_mode=Mode.LANE_DRIVE,
    )
    diagnostics = report["cone_entry_guard"]["diagnostics"]

    assert diagnostics == {
        "first_qualifying_time_s": 1.0,
        "first_qualifying_relative_time_s": pytest.approx(
            0.01,
            abs=TIMING_TOLERANCE_S,
        ),
        "transition_time_s": 1.21,
        "transition_relative_time_s": pytest.approx(
            0.22,
            abs=TIMING_TOLERANCE_S,
        ),
        "trigger_delay_from_first_qualifying_s": pytest.approx(
            0.21,
            abs=TIMING_TOLERANCE_S,
        ),
        "qualifying_unique_messages": 3,
        "absolute_time_shift_invariant": True,
        "course_location_ground_truth": False,
        "timing_interpretation": (
            "bag-relative regression diagnostic only; not a course-location "
            "or race-day cone-location ground truth"
        ),
    }
