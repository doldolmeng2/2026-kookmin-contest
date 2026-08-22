import math

from main.runtime_diagnostics import (
    RuntimeDiagnosticReporter,
    RuntimeDiagnosticSnapshot,
    format_runtime_diagnostic,
)


def snapshot(**changes):
    values = {
        "mode": "LANE_DRIVE",
        "control_source": "LANE",
        "control_reason": "fresh lane command selected",
        "safety_reason": None,
        "missing_inputs": (),
        "stale_inputs": (),
        "traffic_stop_override": False,
        "lane_action_safe_to_drive": False,
        "lane_action_pending": False,
        "lane_action_completed": False,
        "zone_elapsed_s": None,
        "side_obstacle_seen": False,
        "clear_timer_elapsed_s": None,
        "same_lane_brake_reason": "obstacle not confirmed in our lane",
    }
    values.update(changes)
    return RuntimeDiagnosticSnapshot(**values)


def test_changed_signature_logs_immediately_and_duplicate_is_suppressed():
    reporter = RuntimeDiagnosticReporter()
    normal = snapshot()
    assert reporter.update(normal, 1.0) is not None
    assert reporter.update(normal, 1.1) is None
    assert reporter.update(snapshot(lane_action_pending=True), 1.2) is not None


def test_unchanged_hold_is_throttled_to_one_second():
    reporter = RuntimeDiagnosticReporter()
    hold = snapshot(control_source="HOLD", control_reason="safety hold")
    assert reporter.update(hold, 1.0) is not None
    assert reporter.update(hold, 1.5) is None
    assert reporter.update(hold, 2.0) is not None


def test_missing_and_stale_names_are_formatted():
    message = format_runtime_diagnostic(
        snapshot(
            control_source="HOLD",
            control_reason="safety hold",
            safety_reason="required inputs unavailable",
            missing_inputs=("sensor:scan",),
            stale_inputs=("perception:side_clearance",),
        )
    )
    assert "sensor:scan" in message
    assert "perception:side_clearance" in message


def test_traffic_hold_and_safety_hold_have_distinct_signatures():
    traffic = snapshot(
        control_source="HOLD",
        control_reason="recoverable route-traffic hold",
        traffic_stop_override=True,
    )
    safety = snapshot(
        control_source="HOLD",
        control_reason="safety hold",
        safety_reason="stale required inputs: sensor:scan",
        stale_inputs=("sensor:scan",),
    )
    assert traffic.signature != safety.signature
    assert "traffic_stop_override=True" in format_runtime_diagnostic(traffic)
    assert "safety hold" in format_runtime_diagnostic(safety)


def test_formatter_handles_nan_and_infinity_without_crashing():
    message = format_runtime_diagnostic(
        snapshot(zone_elapsed_s=math.nan, clear_timer_elapsed_s=math.inf)
    )
    assert "zone_elapsed=nan" in message
    assert "clear_timer=inf" in message
