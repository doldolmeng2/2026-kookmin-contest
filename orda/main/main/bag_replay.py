"""Read rosbag2 records directly and feed event edges to the pure race FSM.

This module is deliberately independent of the ROS graph.  ROS message support
is imported lazily only when a bag is opened, which also keeps the unit tests
usable without a sourced ROS environment.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
import json
import statistics
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence

from .cone_entry import ConeEntryConfig
from .mission_observation import MissionObservation
from .race_context import RaceContext
from .race_fsm import Mode, RaceFSM, Transition
from .safety_monitor import SafetyMonitor


READ_TOPICS = (
    "/traffic_detection",
    "/rubbercone_info",
    "/lane_offset",
    "/mode_info",
    "/xycar_motor",
    "/scan",
)

EXPECTED_TYPES = {
    "/traffic_detection": "std_msgs/msg/Bool",
    "/rubbercone_info": "std_msgs/msg/Int32MultiArray",
    "/lane_offset": "std_msgs/msg/Int16",
    "/mode_info": "std_msgs/msg/Int32MultiArray",
    "/xycar_motor": "std_msgs/msg/Float32MultiArray",
    "/scan": "sensor_msgs/msg/LaserScan",
}

_FSM_EVENT_TOPICS = frozenset(
    {"/traffic_detection", "/rubbercone_info", "/scan"}
)
_MOTION_MODES = frozenset(
    {
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.REJOIN,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    }
)
_ALLOWED_TRANSITIONS = frozenset(
    {
        (Mode.INIT, Mode.WAIT_GREEN),
        (Mode.WAIT_GREEN, Mode.LANE_DRIVE),
        (Mode.LANE_DRIVE, Mode.CONE_DRIVE),
        (Mode.CONE_DRIVE, Mode.REJOIN),
        *((mode, Mode.STOP) for mode in Mode if mode not in (Mode.FINISH, Mode.STOP)),
    }
)


class BagEnvironmentError(RuntimeError):
    """Raised when direct rosbag reading support is not available."""


@dataclass(frozen=True)
class BagEvent:
    """One serialized bag record after optional message decoding."""

    topic: str
    type_name: str
    timestamp_ns: int
    message: Any = None
    decode_error: Optional[str] = None


class SequentialBagReader:
    """Thin read-only wrapper around ``rosbag2_py.SequentialReader``."""

    def __init__(self, bag_path: Path | str) -> None:
        self.bag_path = Path(bag_path).expanduser().resolve()
        self.topic_types: Dict[str, str] = {}
        self._reader: Any = None
        self._deserialize_message: Any = None
        self._get_message: Any = None

    @staticmethod
    def _storage_id(bag_path: Path) -> str:
        if any(bag_path.glob("*.mcap")):
            return "mcap"
        return "sqlite3"

    def open(self) -> None:
        if not self.bag_path.is_dir():
            raise FileNotFoundError(f"bag directory does not exist: {self.bag_path}")

        try:
            from rclpy.serialization import deserialize_message
            from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise BagEnvironmentError(
                "direct rosbag reading requires a sourced ROS 2 environment "
                "providing rosbag2_py, rclpy.serialization, and "
                "rosidl_runtime_py.utilities"
            ) from exc

        reader = SequentialReader()
        try:
            reader.open(
                StorageOptions(
                    uri=str(self.bag_path),
                    storage_id=self._storage_id(self.bag_path),
                ),
                ConverterOptions("", ""),
            )
        except Exception as exc:
            raise BagEnvironmentError(
                f"failed to open bag directory {self.bag_path}: {exc}"
            ) from exc

        self.topic_types = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
        self._reader = reader
        self._deserialize_message = deserialize_message
        self._get_message = get_message

    def iter_events(self, max_events: Optional[int] = None) -> Iterator[BagEvent]:
        if self._reader is None:
            raise RuntimeError("open() must be called before iter_events()")
        if max_events is not None and max_events < 1:
            raise ValueError("max_events must be positive")

        records_read = 0
        while self._reader.has_next():
            if max_events is not None and records_read >= max_events:
                break

            topic, serialized, timestamp_ns = self._reader.read_next()
            records_read += 1
            type_name = self.topic_types.get(topic, "")

            if topic not in READ_TOPICS:
                yield BagEvent(topic, type_name, timestamp_ns)
                continue

            try:
                message_class = self._get_message(type_name)
                message = self._deserialize_message(serialized, message_class)
            except Exception as exc:
                yield BagEvent(
                    topic,
                    type_name,
                    timestamp_ns,
                    decode_error=f"{type(exc).__name__}: {exc}",
                )
                continue

            yield BagEvent(topic, type_name, timestamp_ns, message)


def _seconds(timestamp_ns: int) -> float:
    return timestamp_ns / 1_000_000_000.0


def _relative_seconds(timestamp_ns: int, first_timestamp_ns: int) -> float:
    return (timestamp_ns - first_timestamp_ns) / 1_000_000_000.0


def _numeric_summary(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"min": None, "max": None, "mean": None}
    return {
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def _gap_summary(timestamp_values_ns: Sequence[int]) -> Dict[str, Optional[float]]:
    gaps = [
        (later - earlier) / 1_000_000_000.0
        for earlier, later in zip(timestamp_values_ns, timestamp_values_ns[1:])
    ]
    if not gaps:
        return {"average_gap_s": None, "maximum_gap_s": None}
    return {
        "average_gap_s": statistics.fmean(gaps),
        "maximum_gap_s": max(gaps),
    }


def _message_data(message: Any) -> Any:
    return getattr(message, "data", None)


def parse_traffic_message(message: Any) -> tuple[Optional[bool], Optional[str]]:
    value = _message_data(message)
    if isinstance(value, bool):
        return value, None
    return None, "traffic message does not contain a Bool data field"


def parse_cone_message(message: Any) -> Dict[str, Any]:
    data = _message_data(message)
    try:
        values = list(data)
    except TypeError:
        values = []

    if len(values) < 2:
        return {
            "offset": None,
            "end_flag": None,
            "confidence": None,
            "field_count": len(values),
            "malformed": True,
        }

    return {
        "offset": int(values[0]),
        "end_flag": int(values[1]),
        "confidence": int(values[2]) if len(values) >= 3 else None,
        "field_count": len(values),
        "malformed": False,
    }


def parse_scalar_number(message: Any) -> Optional[float]:
    value = _message_data(message)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def parse_array_numbers(message: Any) -> Optional[List[float]]:
    data = _message_data(message)
    try:
        values = list(data)
    except TypeError:
        return None
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        return None
    return [float(value) for value in values]


class OfflineBagReplay:
    """Process timestamped records without creating any ROS graph entity."""

    def __init__(
        self,
        *,
        start_mode: Mode = Mode.INIT,
        green_min_consecutive_frames: int = 3,
        green_min_duration_s: float = 0.0,
        cone_entry_config: Optional[ConeEntryConfig] = None,
    ) -> None:
        self.start_mode = Mode(start_mode)
        self.green_min_consecutive_frames = green_min_consecutive_frames
        self.green_min_duration_s = green_min_duration_s
        self.using_default_cone_entry_config = cone_entry_config is None
        self.cone_entry_config = cone_entry_config or ConeEntryConfig()

    def run(
        self,
        events: Iterable[BagEvent],
        *,
        bag_path: Path | str,
        topic_types: Mapping[str, str],
    ) -> Dict[str, Any]:
        fsm = RaceFSM(
            initial_state=self.start_mode,
            green_min_consecutive_frames=self.green_min_consecutive_frames,
            green_min_duration_s=self.green_min_duration_s,
            cone_entry_config=self.cone_entry_config,
        )
        context = RaceContext()
        safety_monitor = SafetyMonitor()

        topic_counts: Counter[str] = Counter()
        seen_event_keys = set()
        first_timestamp_ns: Optional[int] = None
        last_timestamp_ns: Optional[int] = None
        latest_accepted_timestamp_ns: Optional[int] = None
        sensor_receipts: Dict[str, float] = {}
        perception_receipts: Dict[str, float] = {}

        warnings: List[str] = []
        unsupported_transitions: List[Dict[str, Any]] = []
        transitions: List[Dict[str, Any]] = []
        traffic_timeline: List[Dict[str, Any]] = []
        cone_timeline: List[Dict[str, Any]] = []
        lane_timeline: List[Dict[str, Any]] = []
        legacy_timeline: List[Dict[str, Any]] = []
        legacy_changes: List[Dict[str, Any]] = []
        motor_angles: List[float] = []
        motor_speeds: List[float] = []
        scan_timestamps_ns: List[int] = []
        cone_timestamps_ns: List[int] = []
        lane_timestamps_ns: List[int] = []

        duplicate_count = 0
        cone_duplicate_count = 0
        regression_count = 0
        decode_error_count = 0
        accepted_event_count = 0
        fsm_evaluation_count = 0
        malformed_cone_count = 0
        two_field_cone_count = 0
        motor_record_count = 0
        previous_traffic_value: Optional[bool] = None
        previous_legacy_value: Optional[List[float]] = None
        terminal_transition_violation = False
        self_transition_time_violation = False
        transition_log_edge_violation = False
        trace_changed_mode_violation = False

        for event in events:
            topic_counts[event.topic] += 1
            if first_timestamp_ns is None or event.timestamp_ns < first_timestamp_ns:
                first_timestamp_ns = event.timestamp_ns
            if last_timestamp_ns is None or event.timestamp_ns > last_timestamp_ns:
                last_timestamp_ns = event.timestamp_ns

            event_key = (event.topic, event.timestamp_ns)
            if event_key in seen_event_keys:
                duplicate_count += 1
                if event.topic == "/rubbercone_info":
                    cone_duplicate_count += 1
                warnings.append(
                    f"duplicate event rejected: {event.topic} at {event.timestamp_ns} ns"
                )
                continue
            seen_event_keys.add(event_key)

            if (
                latest_accepted_timestamp_ns is not None
                and event.timestamp_ns < latest_accepted_timestamp_ns
            ):
                regression_count += 1
                if (
                    event.topic == "/rubbercone_info"
                    and fsm.state is Mode.LANE_DRIVE
                ):
                    fsm.cone_entry_guard.reject_timestamp_regression()
                warnings.append(
                    "timestamp regression rejected: "
                    f"{event.topic} at {event.timestamp_ns} ns after "
                    f"{latest_accepted_timestamp_ns} ns"
                )
                continue

            latest_accepted_timestamp_ns = event.timestamp_ns
            accepted_event_count += 1

            if event.decode_error is not None:
                decode_error_count += 1
                warnings.append(
                    f"decode failed for {event.topic} at {event.timestamp_ns} ns: "
                    f"{event.decode_error}"
                )
                continue

            if event.topic not in READ_TOPICS:
                continue

            now = _seconds(event.timestamp_ns)
            if context.state_entered_at is None:
                # Establishing the selected initial Mode is initialization, not
                # a Mode transition. Subsequent self-transitions preserve it.
                context.state_entered_at = now

            parsed_traffic: Optional[bool] = None
            parsed_cone: Optional[Dict[str, Any]] = None
            mode_before_trace = fsm.state

            if event.topic == "/traffic_detection":
                perception_receipts["traffic_detection"] = now
                parsed_traffic, warning = parse_traffic_message(event.message)
                green_edge = bool(parsed_traffic) and previous_traffic_value is not True
                traffic_timeline.append(
                    {
                        "_timestamp_ns": event.timestamp_ns,
                        "timestamp_s": now,
                        "relative_time_s": None,
                        "value": parsed_traffic,
                        "green_candidate_edge": green_edge,
                    }
                )
                if warning is not None:
                    warnings.append(
                        f"{warning} at {event.timestamp_ns} ns"
                    )
                if parsed_traffic is not None:
                    previous_traffic_value = parsed_traffic

            elif event.topic == "/rubbercone_info":
                perception_receipts["rubbercone_info"] = now
                parsed_cone = parse_cone_message(event.message)
                cone_timestamps_ns.append(event.timestamp_ns)
                if parsed_cone["field_count"] == 2:
                    two_field_cone_count += 1
                if parsed_cone["malformed"]:
                    malformed_cone_count += 1
                    warnings.append(
                        "malformed rubbercone message rejected as mission evidence: "
                        f"{event.timestamp_ns} ns has "
                        f"{parsed_cone['field_count']} fields"
                    )
                candidate_evidence = bool(
                    not parsed_cone["malformed"]
                    and parsed_cone["end_flag"] == 0
                    and parsed_cone["confidence"] is not None
                    and parsed_cone["confidence"] > 0
                )
                cone_timeline.append(
                    {
                        "_timestamp_ns": event.timestamp_ns,
                        "timestamp_s": now,
                        "relative_time_s": None,
                        **parsed_cone,
                        "candidate_evidence": candidate_evidence,
                        "guard_evaluated": False,
                    }
                )

            elif event.topic == "/lane_offset":
                perception_receipts["lane_offset"] = now
                lane_value = parse_scalar_number(event.message)
                lane_timestamps_ns.append(event.timestamp_ns)
                lane_timeline.append(
                    {
                        "_timestamp_ns": event.timestamp_ns,
                        "timestamp_s": now,
                        "relative_time_s": None,
                        "offset": lane_value,
                    }
                )
                if lane_value is None:
                    warnings.append(
                        f"malformed lane_offset at {event.timestamp_ns} ns"
                    )

            elif event.topic == "/mode_info":
                mode_values = parse_array_numbers(event.message)
                legacy_entry = {
                    "_timestamp_ns": event.timestamp_ns,
                    "timestamp_s": now,
                    "relative_time_s": None,
                    "value": mode_values,
                }
                legacy_timeline.append(legacy_entry)
                if mode_values is None:
                    warnings.append(
                        f"malformed mode_info at {event.timestamp_ns} ns"
                    )
                elif previous_legacy_value != mode_values:
                    legacy_changes.append(dict(legacy_entry))
                    previous_legacy_value = mode_values

            elif event.topic == "/xycar_motor":
                motor_record_count += 1
                motor_values = parse_array_numbers(event.message)
                if motor_values is None or len(motor_values) < 2:
                    warnings.append(
                        f"malformed recorded motor message at {event.timestamp_ns} ns"
                    )
                else:
                    motor_angles.append(motor_values[0])
                    motor_speeds.append(motor_values[1])

            elif event.topic == "/scan":
                sensor_receipts["scan"] = now
                scan_timestamps_ns.append(event.timestamp_ns)

            if event.topic in ("/lane_offset", "/mode_info", "/xycar_motor"):
                if fsm.state is not mode_before_trace:
                    trace_changed_mode_violation = True
                continue

            if not self._should_evaluate(fsm.state, event.topic):
                continue

            observation = MissionObservation(
                now=now,
                sensor_received_at=sensor_receipts,
                perception_received_at=perception_receipts,
                green_detected=(
                    event.topic == "/traffic_detection"
                    and parsed_traffic is True
                ),
                lane_valid=False,
                cone_detected=(
                    event.topic == "/rubbercone_info"
                    and parsed_cone is not None
                    and not parsed_cone["malformed"]
                    and parsed_cone["end_flag"] == 0
                    and parsed_cone["confidence"] is not None
                    and parsed_cone["confidence"] > 0
                ),
                cone_finished=(
                    event.topic == "/rubbercone_info"
                    and parsed_cone is not None
                    and not parsed_cone["malformed"]
                    and parsed_cone["end_flag"] != 0
                ),
                cone_confidence=(
                    parsed_cone["confidence"]
                    if event.topic == "/rubbercone_info"
                    and parsed_cone is not None
                    and not parsed_cone["malformed"]
                    else None
                ),
                cone_end_flag=(
                    bool(parsed_cone["end_flag"])
                    if event.topic == "/rubbercone_info"
                    and parsed_cone is not None
                    and not parsed_cone["malformed"]
                    and parsed_cone["end_flag"] is not None
                    else None
                ),
                cone_message_received_at=(
                    now if event.topic == "/rubbercone_info" else None
                ),
                scan_received_at=sensor_receipts.get("scan"),
            )
            safety = safety_monitor.evaluate(
                fsm.state,
                context,
                observation,
                motion_enabled=fsm.state in _MOTION_MODES,
            )
            entered_before = context.state_entered_at
            state_before = fsm.state
            transition = fsm.step(observation, context, safety)
            fsm_evaluation_count += 1

            if (
                event.topic == "/rubbercone_info"
                and fsm.last_cone_entry_decision is not None
            ):
                decision = fsm.last_cone_entry_decision
                cone_timeline[-1].update(
                    {
                        "guard_evaluated": True,
                        "guard_triggered": decision.triggered,
                        "guard_reason": decision.reason,
                        "guard_new_message": decision.new_message,
                        "guard_sequence_reset": decision.sequence_reset,
                        "guard_first_qualifying_time_s": (
                            decision.first_qualifying_at
                        ),
                        "guard_qualifying_message_count": (
                            decision.qualifying_message_count
                        ),
                        "guard_elapsed_s": decision.elapsed_s,
                    }
                )

            if not transition.changed and context.state_entered_at != entered_before:
                self_transition_time_violation = True
            if state_before in RaceFSM.TERMINAL_STATES and transition.changed:
                terminal_transition_violation = True

            new_logs = 0
            if transition.changed:
                new_logs += 1
                entry = self._transition_entry(
                    transition,
                    event.timestamp_ns,
                )
                transitions.append(entry)
                if (transition.source, transition.target) not in _ALLOWED_TRANSITIONS:
                    unsupported_transitions.append(entry)
            if new_logs > 1:
                transition_log_edge_violation = True

        bag_path = Path(bag_path).expanduser().resolve()
        if first_timestamp_ns is None:
            first_timestamp_ns = 0
        if last_timestamp_ns is None:
            last_timestamp_ns = first_timestamp_ns

        for timeline in (
            transitions,
            traffic_timeline,
            cone_timeline,
            lane_timeline,
            legacy_timeline,
            legacy_changes,
        ):
            for entry in timeline:
                entry["relative_time_s"] = _relative_seconds(
                    entry.pop("_timestamp_ns"), first_timestamp_ns
                )

        metadata_warnings = self._metadata_warnings(topic_types)
        warnings.extend(metadata_warnings)
        missing_topics = [topic for topic in READ_TOPICS if topic not in topic_types]

        confidence_values = [
            entry["confidence"]
            for entry in cone_timeline
            if entry["confidence"] is not None
        ]
        confidence_distribution = Counter(confidence_values)
        end_messages = [
            entry
            for entry in cone_timeline
            if entry["end_flag"] is not None and entry["end_flag"] != 0
        ]
        end_events = []
        previous_end_flag = False
        for entry in cone_timeline:
            current_end_flag = bool(
                entry["end_flag"] is not None and entry["end_flag"] != 0
            )
            if current_end_flag and not previous_end_flag:
                end_events.append(entry)
            if entry["end_flag"] is not None:
                previous_end_flag = current_end_flag
        lane_values = [
            entry["offset"]
            for entry in lane_timeline
            if entry["offset"] is not None
        ]

        cone_entry_transition = next(
            (
                entry
                for entry in transitions
                if entry["source_mode"] == Mode.LANE_DRIVE.value
                and entry["target_mode"] == Mode.CONE_DRIVE.value
            ),
            None,
        )
        cone_entry_trigger = next(
            (
                entry
                for entry in cone_timeline
                if entry.get("guard_triggered") is True
            ),
            None,
        )
        trigger_delay_s = (
            cone_entry_trigger.get("guard_elapsed_s")
            if cone_entry_trigger is not None
            else None
        )
        first_qualifying_relative_time_s = None
        if (
            cone_entry_transition is not None
            and trigger_delay_s is not None
        ):
            first_qualifying_relative_time_s = (
                cone_entry_transition["relative_time_s"] - trigger_delay_s
            )

        invariant_results = {
            "same_topic_timestamp_processed_once": True,
            "accepted_timestamps_monotonic": regression_count == 0,
            "self_transition_preserves_state_entered_at": (
                not self_transition_time_violation
            ),
            "terminal_modes_are_sticky": not terminal_transition_violation,
            "only_allowed_transitions_observed": not unsupported_transitions,
            "at_most_one_transition_log_per_event": (
                not transition_log_edge_violation
            ),
            "lane_and_legacy_traces_do_not_change_mode": (
                not trace_changed_mode_violation
            ),
            "lane_validity_not_inferred": True,
            "recorded_motor_output_not_emitted": True,
            "bag_timestamp_clock_only": True,
            "cone_entry_transition_at_most_once": sum(
                entry["source_mode"] == Mode.LANE_DRIVE.value
                and entry["target_mode"] == Mode.CONE_DRIVE.value
                for entry in transitions
            )
            <= 1,
            "cone_entry_only_uses_lane_to_cone": all(
                entry["source_mode"] == Mode.LANE_DRIVE.value
                and entry["target_mode"] == Mode.CONE_DRIVE.value
                for entry in transitions
                if entry["target_mode"] == Mode.CONE_DRIVE.value
            ),
            "no_cone_entry_after_first_end_flag": all(
                not end_events
                or entry["timestamp_s"] <= end_events[0]["timestamp_s"]
                for entry in transitions
                if entry["target_mode"] == Mode.CONE_DRIVE.value
            ),
        }

        report: Dict[str, Any] = {
            "schema_version": 1,
            "basic_info": {
                "bag_path": str(bag_path),
                "bag_name": bag_path.name,
                "selected_start_mode": self.start_mode.value,
                "duration_s": (last_timestamp_ns - first_timestamp_ns)
                / 1_000_000_000.0,
                "first_bag_timestamp_s": _seconds(first_timestamp_ns),
                "last_bag_timestamp_s": _seconds(last_timestamp_ns),
                "topics": {
                    topic: {
                        "type": type_name,
                        "count": topic_counts.get(topic, 0),
                    }
                    for topic, type_name in sorted(topic_types.items())
                },
                "missing_topics": missing_topics,
            },
            "fsm": {
                "initial_mode": self.start_mode.value,
                "final_mode": fsm.state.value,
                "transition_count": len(transitions),
                "transition_timeline": transitions,
                "fsm_evaluation_count": fsm_evaluation_count,
                "state_entered_at_s": context.state_entered_at,
                "race_started_at_s": context.race_started_at,
                "cone_entered_at_s": context.cone_entered_at,
            },
            "cone_entry_guard": {
                "config": asdict(fsm.cone_entry_config),
                "default_is_provisional": True,
                "using_default_config": self.using_default_cone_entry_config,
                "warning": (
                    "positive-bag provisional default, not a final threshold; "
                    "revalidate after collecting negative processed bags"
                ),
                "diagnostics": {
                    "first_qualifying_time_s": (
                        cone_entry_trigger.get("guard_first_qualifying_time_s")
                        if cone_entry_trigger is not None
                        else None
                    ),
                    "first_qualifying_relative_time_s": (
                        first_qualifying_relative_time_s
                    ),
                    "transition_time_s": (
                        cone_entry_transition["timestamp_s"]
                        if cone_entry_transition is not None
                        else None
                    ),
                    "transition_relative_time_s": (
                        cone_entry_transition["relative_time_s"]
                        if cone_entry_transition is not None
                        else None
                    ),
                    "trigger_delay_from_first_qualifying_s": trigger_delay_s,
                    "qualifying_unique_messages": (
                        cone_entry_trigger.get(
                            "guard_qualifying_message_count"
                        )
                        if cone_entry_trigger is not None
                        else 0
                    ),
                    "absolute_time_shift_invariant": True,
                    "course_location_ground_truth": False,
                    "timing_interpretation": (
                        "bag-relative regression diagnostic only; not a "
                        "course-location or race-day cone-location ground truth"
                    ),
                },
            },
            "traffic": {
                "timeline": traffic_timeline,
                "green_candidate_edges": [
                    entry
                    for entry in traffic_timeline
                    if entry["green_candidate_edge"]
                ],
                "used_only_as_start_green_candidate": True,
                "used_as_left_turn_signal": False,
            },
            "rubbercone": {
                "timeline": cone_timeline,
                "cone_candidate_timeline": [
                    entry for entry in cone_timeline if entry["candidate_evidence"]
                ],
                "statistics": {
                    "message_count": len(cone_timeline),
                    "confidence": {
                        **_numeric_summary(confidence_values),
                        "distribution": {
                            str(value): count
                            for value, count in sorted(
                                confidence_distribution.items()
                            )
                        },
                    },
                    "first_end_flag_timestamp_s": (
                        end_events[0]["timestamp_s"] if end_events else None
                    ),
                    "first_end_flag_relative_time_s": (
                        end_events[0]["relative_time_s"] if end_events else None
                    ),
                    "end_flag_event_count": len(end_events),
                    "end_flag_message_count": len(end_messages),
                    **_gap_summary(cone_timestamps_ns),
                    "same_timestamp_duplicate_count": cone_duplicate_count,
                    "two_field_message_count": two_field_cone_count,
                    "malformed_message_count": malformed_cone_count,
                },
            },
            "lane_reference": {
                "timeline": lane_timeline,
                "statistics": {
                    "message_count": len(lane_timeline),
                    **_numeric_summary(lane_values),
                    **_gap_summary(lane_timestamps_ns),
                },
                "lane_validity_inferred": False,
                "warning": (
                    "lane_offset is a reference trace only; no lane validity "
                    "or detector quality was inferred"
                ),
            },
            "legacy_reference": {
                "mode_info_timeline": legacy_timeline,
                "legacy_mode_changes": legacy_changes,
                "used_as_new_mode_label": False,
                "warning": (
                    "legacy mode_info was not used as a label or a Mode input"
                ),
            },
            "recorded_motor_reference": {
                "recorded_message_count": motor_record_count,
                "angle": _numeric_summary(motor_angles),
                "speed": _numeric_summary(motor_speeds),
                "ros_output_attempted": False,
            },
            "scan_reference": {
                "message_count": len(scan_timestamps_ns),
                **_gap_summary(scan_timestamps_ns),
                "used_for_perception_rerun": False,
            },
            "processing": {
                "raw_record_count": sum(topic_counts.values()),
                "accepted_record_count": accepted_event_count,
                "duplicate_record_count": duplicate_count,
                "timestamp_regression_count": regression_count,
                "decode_error_count": decode_error_count,
            },
            "validation": {
                "invariant_pass": all(invariant_results.values()),
                "invariants": invariant_results,
                "warnings": warnings,
                "unsupported_transitions": unsupported_transitions,
                "verifiable_scope": self._verifiable_scope(topic_types),
                "unverifiable_scope": self._unverifiable_scope(topic_types),
            },
        }
        return report

    @staticmethod
    def _should_evaluate(mode: Mode, topic: str) -> bool:
        # Trace-only topics do not create mission event edges. In WAIT_GREEN,
        # only a new traffic record may advance/reset the green debouncer; this
        # prevents one cached Bool from being counted again by unrelated data.
        if topic not in _FSM_EVENT_TOPICS:
            return False
        if mode is Mode.WAIT_GREEN:
            return topic == "/traffic_detection"
        return True

    @staticmethod
    def _transition_entry(
        transition: Transition,
        timestamp_ns: int,
    ) -> Dict[str, Any]:
        return {
            "_timestamp_ns": timestamp_ns,
            "timestamp_s": _seconds(timestamp_ns),
            "relative_time_s": None,
            "source_mode": transition.source.value,
            "target_mode": transition.target.value,
            "reason": transition.reason,
        }

    @staticmethod
    def _metadata_warnings(topic_types: Mapping[str, str]) -> List[str]:
        warnings = []
        for topic in READ_TOPICS:
            if topic not in topic_types:
                warnings.append(f"missing topic: {topic}")
                continue
            expected = EXPECTED_TYPES[topic]
            actual = topic_types[topic]
            if actual != expected:
                warnings.append(
                    f"unexpected type for {topic}: expected {expected}, got {actual}"
                )
        return warnings

    @staticmethod
    def _verifiable_scope(topic_types: Mapping[str, str]) -> List[str]:
        scope = [
            "timestamp ordering, duplicate rejection, and terminal Mode behavior",
            "absence of unsupported Mode transitions",
        ]
        if "/traffic_detection" in topic_types:
            scope.append("WAIT_GREEN debounce using recorded traffic Bool edges")
        if "/rubbercone_info" in topic_types:
            scope.append("rubbercone candidate/end event timing")
        if "/rubbercone_info" in topic_types and "/scan" in topic_types:
            scope.append(
                "provisional LANE_DRIVE to CONE_DRIVE guard and transition timing"
            )
        if "/rubbercone_info" in topic_types:
            scope.append(
                "CONE_DRIVE to REJOIN on a fresh recorded rubbercone 0-to-1 session"
            )
        if "/scan" in topic_types:
            scope.append("recorded scan receipt gap statistics")
        return scope

    @staticmethod
    def _unverifiable_scope(topic_types: Mapping[str, str]) -> List[str]:
        scope = [
            "REJOIN to FIXED_AVOID",
            "fixed-vehicle avoidance and moving-vehicle overtake",
            "lap/Gate counting and FINISH entry",
            "left-turn recognition, shortcut readiness, and SHORTCUT entry",
            "lane validity or lane detector performance from lane_offset",
            "motor adapter behavior or live vehicle motion",
        ]
        if "/traffic_detection" not in topic_types:
            scope.append("recorded start-green debounce")
        if "/rubbercone_info" not in topic_types:
            scope.append("rubbercone candidate and end event timing")
        return scope


def write_report_files(
    report: Mapping[str, Any],
    output_dir: Path | str,
) -> tuple[Path, Path]:
    basic = report["basic_info"]
    mode_dir = Path(output_dir).expanduser() / basic["selected_start_mode"]
    mode_dir.mkdir(parents=True, exist_ok=True)
    base_name = basic["bag_name"]
    json_path = mode_dir / f"{base_name}.json"
    text_path = mode_dir / f"{base_name}.txt"

    json_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    text_path.write_text(format_text_report(report), encoding="utf-8")
    return json_path, text_path


def format_text_report(report: Mapping[str, Any]) -> str:
    basic = report["basic_info"]
    fsm = report["fsm"]
    cone_guard = report["cone_entry_guard"]
    cone_stats = report["rubbercone"]["statistics"]
    lane = report["lane_reference"]
    legacy = report["legacy_reference"]
    motor = report["recorded_motor_reference"]
    validation = report["validation"]

    lines = [
        "Offline FSM bag replay report",
        f"bag: {basic['bag_path']}",
        f"selected start Mode: {basic['selected_start_mode']}",
        f"duration: {basic['duration_s']:.9f} s",
        f"first timestamp: {basic['first_bag_timestamp_s']:.9f} s",
        f"last timestamp: {basic['last_bag_timestamp_s']:.9f} s",
        "",
        "Topics",
    ]
    for topic, details in basic["topics"].items():
        lines.append(f"- {topic}: {details['type']}, count={details['count']}")
    lines.append(f"- missing: {', '.join(basic['missing_topics']) or 'none'}")

    lines.extend(
        [
            "",
            "New FSM",
            f"- initial Mode: {fsm['initial_mode']}",
            f"- final Mode: {fsm['final_mode']}",
            f"- transition count: {fsm['transition_count']}",
            f"- evaluation count: {fsm['fsm_evaluation_count']}",
            f"- cone entered at: {fsm['cone_entered_at_s']}",
        ]
    )
    for item in fsm["transition_timeline"]:
        lines.append(
            "- transition "
            f"t={item['relative_time_s']:.9f}s "
            f"{item['source_mode']} -> {item['target_mode']}: {item['reason']}"
        )

    lines.extend(
        [
            "",
            "Cone-entry guard",
            f"- config: {cone_guard['config']}",
            f"- default is provisional: {cone_guard['default_is_provisional']}",
            f"- using default config: {cone_guard['using_default_config']}",
            f"- warning: {cone_guard['warning']}",
            f"- diagnostics: {cone_guard['diagnostics']}",
            "- transition timing is a bag-relative regression diagnostic only",
            "- course/race-day cone location ground truth: false",
        ]
    )

    lines.extend(["", "Traffic timeline"])
    for item in report["traffic"]["timeline"]:
        lines.append(
            f"- t={item['relative_time_s']:.9f}s value={item['value']} "
            f"green_edge={item['green_candidate_edge']}"
        )

    lines.extend(
        [
            "",
            "Rubbercone statistics",
            f"- message count: {cone_stats['message_count']}",
            f"- confidence: {cone_stats['confidence']}",
            f"- first end flag: {cone_stats['first_end_flag_relative_time_s']}",
            f"- end flag count: {cone_stats['end_flag_event_count']}",
            f"- end flag message count: {cone_stats['end_flag_message_count']}",
            f"- average gap: {cone_stats['average_gap_s']}",
            f"- maximum gap: {cone_stats['maximum_gap_s']}",
            f"- duplicate timestamp count: {cone_stats['same_timestamp_duplicate_count']}",
            f"- two-field count: {cone_stats['two_field_message_count']}",
            f"- malformed count: {cone_stats['malformed_message_count']}",
            "",
            "Rubbercone timeline",
        ]
    )
    for item in report["rubbercone"]["timeline"]:
        lines.append(
            f"- t={item['relative_time_s']:.9f}s offset={item['offset']} "
            f"end={item['end_flag']} confidence={item['confidence']} "
            f"malformed={item['malformed']} candidate={item['candidate_evidence']} "
            f"guard={item.get('guard_reason', 'not evaluated')}"
        )

    lane_stats = lane["statistics"]
    lines.extend(
        [
            "",
            "Lane reference trace",
            f"- count/min/max/mean: {lane_stats['message_count']}/"
            f"{lane_stats['min']}/{lane_stats['max']}/{lane_stats['mean']}",
            f"- average/maximum gap: {lane_stats['average_gap_s']}/"
            f"{lane_stats['maximum_gap_s']}",
            f"- warning: {lane['warning']}",
        ]
    )
    for item in lane["timeline"]:
        lines.append(
            f"- lane t={item['relative_time_s']:.9f}s offset={item['offset']}"
        )

    lines.extend(["", "Legacy mode reference trace", f"- warning: {legacy['warning']}"])
    for item in legacy["mode_info_timeline"]:
        lines.append(
            f"- mode t={item['relative_time_s']:.9f}s value={item['value']}"
        )

    lines.extend(
        [
            "",
            "Recorded motor reference",
            f"- count: {motor['recorded_message_count']}",
            f"- angle: {motor['angle']}",
            f"- speed: {motor['speed']}",
            f"- ROS output attempted: {motor['ros_output_attempted']}",
            "",
            "Validation",
            f"- all invariants pass: {validation['invariant_pass']}",
        ]
    )
    for name, passed in validation["invariants"].items():
        lines.append(f"- {name}: {'PASS' if passed else 'FAIL'}")
    lines.append("- warnings:")
    lines.extend(f"  - {warning}" for warning in validation["warnings"])
    lines.append("- unsupported transitions:")
    lines.extend(
        f"  - {entry['source_mode']} -> {entry['target_mode']}"
        for entry in validation["unsupported_transitions"]
    )
    lines.append("- verifiable scope:")
    lines.extend(f"  - {item}" for item in validation["verifiable_scope"])
    lines.append("- unverifiable scope:")
    lines.extend(f"  - {item}" for item in validation["unverifiable_scope"])
    return "\n".join(lines) + "\n"
