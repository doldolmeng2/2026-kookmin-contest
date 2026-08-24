"""Which recorded topics a replay is allowed to publish.

Pure module — no ROS imports — so the selection policy is unit-testable without a
sourced workspace.

The vehicle is driven by two different command layers and it matters which one a
replay reproduces:

``/joy`` -> ``/xycar_motor``
    The joystick node turns raw joycon axes into ``[steering_deg, speed]``.  Both
    topics are *joycon-manipulation output* and are therefore refused by default:
    replaying them means driving the car from the recorded stick, and re-deriving
    ``/xycar_motor`` from a replayed ``/joy`` additionally re-phases the 20 Hz
    manual-drive timer, which destroys the timing match.

``/commands/servo/position`` + ``/commands/motor/speed``
    What the VESC driver actually put on the wire: servo position (steering) and
    eRPM (speed), already clipped to the vehicle limits.  These are the actuation
    the car physically obeyed, so replaying them reproduces the recorded motion
    without using the joycon steering/speed values.

``/sensors/servo_position_command`` is the driver's echo of the accepted servo
position.  It is part of the recorded actuation stream and is replayed with the
rest, but it is a driver *output*: on a live vehicle where ``vesc_driver`` is
running, list it in ``exclude_topics`` so the replay does not double-publish it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple


#: Servo/eRPM commands the VESC driver consumed, plus its own echo of the servo
#: value.  This is the actuation layer that moved the car.
ACTUATION_TOPICS: Tuple[str, ...] = (
    '/commands/servo/position',
    '/commands/motor/speed',
    '/sensors/servo_position_command',
)

#: Driver outputs that a live ``vesc_driver`` also publishes.  Replaying these on
#: a live vehicle creates a second publisher on the topic.
DRIVER_ECHO_TOPICS: Tuple[str, ...] = ('/sensors/servo_position_command',)

#: Driving-mode interface published by ``main_node``.
MODE_TOPICS: Tuple[str, ...] = ('/mode_info', '/lane_info')

#: Joycon-manipulation output.  Refused unless ``allow_joycon`` is set.
JOYCON_TOPICS: Tuple[str, ...] = ('/joy', '/xycar_motor')

#: Raw sensor streams, for feeding a live perception stack instead of driving.
SENSOR_TOPICS: Tuple[str, ...] = (
    '/scan',
    '/xycar_ultrasonic',
    '/imu',
    '/resized_image',
    '/image_raw',
    '/image_raw/compressed',
    '/camera_info',
    '/point_cloud',
)

#: Infrastructure topics that are never replayed: re-publishing them confuses the
#: live ROS graph rather than the vehicle.
NEVER_REPLAY_TOPICS: Tuple[str, ...] = (
    '/rosout',
    '/parameter_events',
    '/events/write_split',
    '/clock',
)

#: What each topic contributes to the recorded motion, used for the run summary.
TOPIC_ROLES: Dict[str, str] = {
    '/commands/servo/position': 'steering',
    '/sensors/servo_position_command': 'steering (driver echo)',
    '/commands/motor/speed': 'speed',
    '/xycar_motor': 'steering+speed (joycon)',
    '/joy': 'raw joycon axes',
    '/mode_info': 'drive mode',
    '/lane_info': 'drive mode (lane)',
}

TOPIC_SETS: Dict[str, Tuple[str, ...]] = {
    # Default: reproduce the motion from the actuation layer, never the joycon.
    'actuation': ACTUATION_TOPICS + MODE_TOPICS,
    # Opt-in: the joystick node's [steering, speed] command.
    'motor': JOYCON_TOPICS[1:] + MODE_TOPICS,
    # Opt-in: every command layer at once.
    'full': ACTUATION_TOPICS + JOYCON_TOPICS[1:] + MODE_TOPICS,
    # Feed a live perception stack instead of driving.
    'sensors': SENSOR_TOPICS,
    # Everything the bag holds except infrastructure (and joycon, unless allowed).
    'all': (),
}


class TopicSelectionError(ValueError):
    """Raised when the requested selection cannot be honoured."""


@dataclass(frozen=True)
class Selection:
    """Result of resolving a requested topic set against one bag."""

    topics: Tuple[str, ...]
    missing: Tuple[str, ...] = ()
    refused_joycon: Tuple[str, ...] = ()
    notes: Tuple[str, ...] = ()

    def role_of(self, topic: str) -> str:
        return TOPIC_ROLES.get(topic, 'other')


def _dedupe(items: Iterable[str]) -> List[str]:
    seen = set()
    out = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def resolve_selection(
    available: Sequence[str],
    topic_set: str = 'actuation',
    include_topics: Sequence[str] = (),
    exclude_topics: Sequence[str] = (),
    allow_joycon: bool = False,
) -> Selection:
    """Resolve the topics to replay.

    ``available`` is the topic list of the bag.  ``include_topics`` is added on top
    of ``topic_set`` (and may name a topic outside any set); ``exclude_topics``
    wins over both.  Topics from the requested set that the bag does not hold are
    reported in :attr:`Selection.missing` rather than raising, because a bag
    legitimately lacks e.g. ``/mode_info`` when it was recorded during a manual run.

    Raises :class:`TopicSelectionError` only when the result would be empty, or when
    the caller explicitly asked for a topic the bag does not have.
    """
    if topic_set not in TOPIC_SETS:
        raise TopicSelectionError(
            'unknown topic_set %r; known sets: %s'
            % (topic_set, ', '.join(sorted(TOPIC_SETS)))
        )

    available_set = set(available)
    excluded = set(exclude_topics)

    if topic_set == 'all':
        requested = [t for t in available if t not in NEVER_REPLAY_TOPICS]
    else:
        requested = list(TOPIC_SETS[topic_set])

    explicit = [t for t in include_topics if t]
    unknown_explicit = [t for t in explicit if t not in available_set]
    if unknown_explicit:
        raise TopicSelectionError(
            'bag does not contain explicitly requested topic(s): %s'
            % ', '.join(sorted(unknown_explicit))
        )

    requested = _dedupe(requested + explicit)

    notes: List[str] = []
    refused: List[str] = []
    if not allow_joycon:
        named_joycon = [t for t in explicit if t in JOYCON_TOPICS]
        if named_joycon:
            raise TopicSelectionError(
                'refusing joycon-derived topic(s) %s: the recorded steering/speed '
                'they carry comes straight from the stick. Pass allow_joycon:=true '
                'to override.' % ', '.join(sorted(named_joycon))
            )
        refused = [t for t in requested if t in JOYCON_TOPICS]
        requested = [t for t in requested if t not in JOYCON_TOPICS]
        if refused:
            notes.append(
                'joycon-derived topic(s) %s not replayed (allow_joycon is false)'
                % ', '.join(refused)
            )

    missing = tuple(t for t in requested if t not in available_set)
    selected = [t for t in requested if t in available_set and t not in excluded]

    if excluded:
        dropped = [t for t in requested if t in excluded and t in available_set]
        if dropped:
            notes.append('excluded by request: %s' % ', '.join(dropped))

    if not selected:
        reason = '; '.join(notes) if notes else 'none of them are in this bag'
        raise TopicSelectionError(
            'no replayable topic left for topic_set=%r (%s); bag holds: %s'
            % (topic_set, reason, ', '.join(sorted(available_set)))
        )

    return Selection(
        topics=tuple(selected),
        missing=missing,
        refused_joycon=tuple(refused),
        notes=tuple(notes),
    )


def describe_selection(selection: Selection) -> str:
    """Human-readable summary of what a replay will and will not drive."""
    lines = ['replaying %d topic(s):' % len(selection.topics)]
    for topic in selection.topics:
        lines.append('  %-34s %s' % (topic, selection.role_of(topic)))
    if selection.missing:
        lines.append('not in this bag (nothing published for them):')
        for topic in selection.missing:
            lines.append(
                '  %-34s %s'
                % (topic, TOPIC_ROLES.get(topic, 'other'))
            )
    for note in selection.notes:
        lines.append('note: %s' % note)
    return '\n'.join(lines)
