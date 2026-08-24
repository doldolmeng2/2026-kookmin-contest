"""Publish a recorded drive command stream back onto the ROS graph.

What the node guarantees, and how:

*content*
    The CDR payload recorded in the bag is handed to ``publish()`` unchanged
    (``rclpy`` publishes ``bytes`` through ``publish_raw``).  No message is
    deserialized, so no float is rounded and no header is rewritten — a subscriber
    receives the same bytes the recorder wrote.

*timing*
    Deadlines are absolute offsets from one playback epoch, so a late publish
    never pushes the next one.  ``timing_mode:=wall`` hits those deadlines on the
    monotonic clock (sleep, then spin — see :mod:`.pacing`) and reports the
    residual error.  ``timing_mode:=sim`` instead drives ``/clock``, which makes
    the error exactly zero in ROS time for every subscriber running with
    ``use_sim_time:=true``.

*completeness*
    The schedule is fully enumerated before the first publish and the run is only
    finished when every entry has gone out.  A publish that raises is retried, and
    the first Ctrl-C is refused with an explanation; a second one within
    ``abort_grace`` seconds aborts and (by default) sends a stop command first.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
import threading
import time

import rclpy
from rclpy.node import Node

from .cue import CueData, CueError, load_or_build_cue, summarize_cue
from .bagio import BagError, open_bag
from .pacing import DeadlinePacer, GcPause, try_realtime_priority
from .qos import merge_offered_qos, parse_offered_qos, to_qos_profile, describe as describe_qos
from .report import ReplayReport, TimingStats
from .schedule import (
    Schedule,
    ScheduleError,
    build_schedule,
    describe_schedule,
    infinite_loop_requested,
)
from .topic_sets import (
    DRIVER_ECHO_TOPICS,
    TopicSelectionError,
    describe_selection,
    resolve_selection,
)

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'

#: Neutral command per topic, sent only when a run is force-aborted so the
#: vehicle does not keep the last replayed throttle.  Values are the CDR encoding
#: of the neutral message, filled in lazily from the message type.
SAFE_STOP_VALUES: Dict[str, float] = {
    '/commands/motor/speed': 0.0,
    '/xycar_motor': 0.0,
}


class ReplayNode(Node):
    """Replay the recorded command topics of one bag."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__('bag_motion_replay', **node_kwargs)

        self.declare_parameter('bag', DEFAULT_BAG)
        self.declare_parameter('cue', '')
        self.declare_parameter('rebuild_cue', False)
        self.declare_parameter('topic_set', 'actuation')
        self.declare_parameter('include_topics', [''])
        self.declare_parameter('exclude_topics', [''])
        self.declare_parameter('allow_joycon', False)
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('start_offset', 0.0)
        self.declare_parameter('end_offset', -1.0)
        self.declare_parameter('loop', 1)
        self.declare_parameter('loop_gap', 0.0)
        self.declare_parameter('timing_mode', 'wall')
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('clock_period_ms', 10.0)
        self.declare_parameter('spin_margin_ms', 1.5)
        self.declare_parameter('realtime_priority', 0)
        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('wait_for_subscribers', False)
        self.declare_parameter('wait_timeout', 15.0)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('publish_retries', 3)
        self.declare_parameter('abort_grace', 3.0)
        self.declare_parameter('safe_stop_on_abort', True)
        self.declare_parameter('report_path', '')
        self.declare_parameter('fail_on_timing_error_ms', 0.0)
        self.declare_parameter('shutdown_when_done', True)

        self.bag_path = str(self.get_parameter('bag').value)
        self.topic_set = str(self.get_parameter('topic_set').value)
        self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        self.rate = float(self.get_parameter('rate').value)
        self.timing_mode = str(self.get_parameter('timing_mode').value).lower()
        if self.timing_mode not in ('wall', 'sim'):
            raise ValueError(
                'timing_mode must be "wall" or "sim", got %r' % self.timing_mode
            )

        self._stop_requested = threading.Event()
        self._abort = threading.Event()
        self._first_interrupt_at: Optional[float] = None
        self._done = threading.Event()
        self.report = ReplayReport()
        self.exit_code = 0

        self.cue: Optional[CueData] = None
        self.schedule: Optional[Schedule] = None
        self.publishers_by_index: List[object] = []
        self.clock_publisher = None

        self._prepare()

    # ------------------------------------------------------------------ setup

    def _string_list(self, name: str) -> List[str]:
        raw = self.get_parameter(name).value or []
        return [str(item) for item in raw if str(item).strip()]

    def _prepare(self) -> None:
        log = self.get_logger()

        try:
            source = open_bag(self.bag_path)
            available = sorted(source.topics())
        except BagError as exc:
            raise SystemExit('cannot open bag: %s' % exc) from exc

        try:
            selection = resolve_selection(
                available=available,
                topic_set=self.topic_set,
                include_topics=self._string_list('include_topics'),
                exclude_topics=self._string_list('exclude_topics'),
                allow_joycon=bool(self.get_parameter('allow_joycon').value),
            )
        except TopicSelectionError as exc:
            raise SystemExit('topic selection failed: %s' % exc) from exc

        for line in describe_selection(selection).splitlines():
            log.info(line)

        if not self.topic_prefix:
            live_echo = [t for t in selection.topics if t in DRIVER_ECHO_TOPICS]
            if live_echo:
                log.warning(
                    'replaying driver-output topic(s) %s: on a vehicle where '
                    'vesc_driver is running this creates a second publisher. Add '
                    'them to exclude_topics for a live run.' % ', '.join(live_echo)
                )

        try:
            cue, cue_path = load_or_build_cue(
                bag_path=self.bag_path,
                topic_names=selection.topics,
                cue_path=str(self.get_parameter('cue').value) or None,
                rebuild=bool(self.get_parameter('rebuild_cue').value),
                progress=self._log_cue_progress,
                log=log.info,
            )
        except (BagError, CueError) as exc:
            raise SystemExit('cannot build replay cue: %s' % exc) from exc

        self.cue = cue
        for line in summarize_cue(cue).splitlines():
            log.info(line)

        end_offset = float(self.get_parameter('end_offset').value)
        loop = int(self.get_parameter('loop').value)
        try:
            self.schedule = build_schedule(
                records=cue.records,
                rate=self.rate,
                start_offset_s=float(self.get_parameter('start_offset').value),
                end_offset_s=None if end_offset < 0.0 else end_offset,
                loop_count=1 if infinite_loop_requested(loop) else max(1, loop),
                loop_gap_s=float(self.get_parameter('loop_gap').value),
            )
        except ScheduleError as exc:
            raise SystemExit('cannot build schedule: %s' % exc) from exc

        for line in describe_schedule(self.schedule, cue.topic_names()).splitlines():
            log.info(line)

        self._create_publishers()

        self.report.bag = str(Path(self.bag_path).expanduser())
        self.report.cue = str(cue_path)
        self.report.topic_set = self.topic_set
        self.report.topics = list(cue.topic_names())
        self.report.rate = self.rate
        self.report.timing_mode = self.timing_mode
        self.report.loop_count = self.schedule.loop_count
        self.report.scheduled = len(self.schedule)
        self.report.bag_duration_s = cue.duration_ns() / 1e9
        self.report.missing_topics = list(selection.missing)
        self.report.notes = list(selection.notes)

    def _log_cue_progress(self, done: int, expected: int) -> None:
        if expected > 0 and done % 20000 == 0:
            self.get_logger().info(
                'cue build: %d / ~%d records read' % (done, expected)
            )

    def _create_publishers(self) -> None:
        from rosidl_runtime_py.utilities import get_message

        depth = int(self.get_parameter('qos_depth').value)
        self.publishers_by_index = []
        for topic in self.cue.topics:
            spec = merge_offered_qos(parse_offered_qos(topic.offered_qos_profiles))
            profile = to_qos_profile(spec, default_depth=depth)
            name = self.topic_prefix + topic.name if self.topic_prefix else topic.name
            message_type = get_message(topic.type_name)
            publisher = self.create_publisher(message_type, name, profile)
            self.publishers_by_index.append(publisher)
            self.get_logger().info(
                'publisher %-34s %-32s %s'
                % (name, topic.type_name, describe_qos(spec))
            )

        if self.timing_mode == 'sim' and bool(self.get_parameter('publish_clock').value):
            from rosgraph_msgs.msg import Clock
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

            clock_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.clock_publisher = self.create_publisher(Clock, '/clock', clock_qos)
            self.get_logger().info(
                'timing_mode=sim: driving /clock; start subscribers with '
                'use_sim_time:=true for an exactly-zero timing error'
            )

    # -------------------------------------------------------------- lifecycle

    def request_stop(self) -> str:
        """Handle an interrupt without silently truncating the run."""
        now = time.monotonic()
        grace = float(self.get_parameter('abort_grace').value)
        if self._first_interrupt_at is not None and now - self._first_interrupt_at <= grace:
            self._abort.set()
            self._stop_requested.set()
            return 'aborting'
        self._first_interrupt_at = now
        return 'refused'

    def wait_for_subscribers(self) -> None:
        if not bool(self.get_parameter('wait_for_subscribers').value):
            return
        timeout = float(self.get_parameter('wait_timeout').value)
        deadline = time.monotonic() + timeout
        pending = list(zip(self.cue.topics, self.publishers_by_index))
        while time.monotonic() < deadline and not self._abort.is_set():
            pending = [
                (topic, publisher)
                for topic, publisher in pending
                if publisher.get_subscription_count() == 0
            ]
            if not pending:
                self.get_logger().info('all replay topics have a subscriber')
                return
            time.sleep(0.05)
        if pending:
            names = ', '.join(topic.name for topic, _ in pending)
            self.get_logger().warning(
                'no subscriber on %s after %.1f s; publishing anyway' % (names, timeout)
            )

    # ---------------------------------------------------------------- publish

    def _safe_stop(self) -> None:
        if not bool(self.get_parameter('safe_stop_on_abort').value):
            return
        from rosidl_runtime_py.utilities import get_message

        for topic, publisher in zip(self.cue.topics, self.publishers_by_index):
            if topic.name not in SAFE_STOP_VALUES:
                continue
            try:
                message_type = get_message(topic.type_name)
                message = message_type()
                if hasattr(message, 'data'):
                    if isinstance(message.data, float):
                        message.data = float(SAFE_STOP_VALUES[topic.name])
                    else:
                        message.data = type(message.data)([0.0, 0.0])
                publisher.publish(message)
                self.get_logger().warning('sent stop command on %s' % topic.name)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().error(
                    'could not send stop command on %s: %s' % (topic.name, exc)
                )

    def _publish_clock(self, ros_time_ns: int) -> None:
        if self.clock_publisher is None:
            return
        from rosgraph_msgs.msg import Clock

        message = Clock()
        message.clock.sec = int(ros_time_ns // 1_000_000_000)
        message.clock.nanosec = int(ros_time_ns % 1_000_000_000)
        self.clock_publisher.publish(message)

    def run_playback(self) -> None:
        """Publish the whole schedule, then report."""
        try:
            self._run_playback()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error('replay failed: %s' % exc)
            self.report.aborted = True
            self.report.abort_reason = str(exc)
            self.exit_code = 1
        finally:
            self._finish()

    def _run_playback(self) -> None:
        log = self.get_logger()
        pacer = DeadlinePacer(
            spin_margin_ns=int(
                float(self.get_parameter('spin_margin_ms').value) * 1e6
            )
        )
        priority = int(self.get_parameter('realtime_priority').value)
        self.report.scheduler = (
            try_realtime_priority(priority) if priority > 0 else 'default scheduler'
        )
        log.info('scheduler: %s' % self.report.scheduler)

        self.wait_for_subscribers()

        start_delay = float(self.get_parameter('start_delay').value)
        if start_delay > 0.0:
            log.info('starting in %.2f s' % start_delay)
            time.sleep(start_delay)

        loop_param = int(self.get_parameter('loop').value)
        repeat_forever = infinite_loop_requested(loop_param)
        retries = max(0, int(self.get_parameter('publish_retries').value))
        clock_period_ns = int(
            float(self.get_parameter('clock_period_ms').value) * 1e6
        )

        errors: List[float] = []
        per_topic_errors: Dict[str, List[float]] = {
            topic.name: [] for topic in self.cue.topics
        }

        pass_index = 0
        wall_start = time.monotonic()
        with GcPause():
            while True:
                epoch_ns = time.monotonic_ns()
                ros_epoch_ns = time.time_ns()
                self._play_once(
                    pacer=pacer,
                    epoch_ns=epoch_ns,
                    ros_epoch_ns=ros_epoch_ns,
                    clock_period_ns=clock_period_ns,
                    retries=retries,
                    errors=errors,
                    per_topic_errors=per_topic_errors,
                )
                pass_index += 1
                if self._abort.is_set():
                    break
                if not repeat_forever:
                    break
                gap = float(self.get_parameter('loop_gap').value)
                log.info('pass %d finished; repeating after %.2f s' % (pass_index, gap))
                if gap > 0.0:
                    time.sleep(gap)

        self.report.wall_duration_s = time.monotonic() - wall_start
        self.report.loop_count = pass_index * self.schedule.loop_count
        self.report.overall = TimingStats.from_errors(errors)
        self.report.per_topic = {
            name: TimingStats.from_errors(values)
            for name, values in per_topic_errors.items()
            if values
        }

    def _play_once(
        self,
        pacer: DeadlinePacer,
        epoch_ns: int,
        ros_epoch_ns: int,
        clock_period_ns: int,
        retries: int,
        errors: List[float],
        per_topic_errors: Dict[str, List[float]],
    ) -> None:
        records = self.cue.records
        publishers = self.publishers_by_index
        topic_names = self.cue.topic_names()
        sim = self.clock_publisher is not None
        next_tick_ns = epoch_ns

        for entry in self.schedule.entries:
            deadline_ns = epoch_ns + entry.delay_ns

            if sim:
                while next_tick_ns < deadline_ns:
                    pacer.wait_until(next_tick_ns, self._abort.is_set)
                    if self._abort.is_set():
                        break
                    self._publish_clock(ros_epoch_ns + (next_tick_ns - epoch_ns))
                    next_tick_ns += clock_period_ns

            released_ns = pacer.wait_until(deadline_ns, self._abort.is_set)

            if self._abort.is_set():
                self.report.aborted = True
                self.report.abort_reason = 'operator abort (second interrupt)'
                self.report.skipped = self.report.scheduled - self.report.published
                self._safe_stop()
                return

            if sim:
                self._publish_clock(ros_epoch_ns + entry.delay_ns)
                next_tick_ns = max(next_tick_ns, deadline_ns + clock_period_ns)

            payload = records[entry.record_index][2]
            publisher = publishers[entry.topic_index]

            published = False
            for attempt in range(retries + 1):
                try:
                    publisher.publish(payload)
                    published = True
                    break
                except Exception as exc:  # pragma: no cover - middleware failure
                    if attempt == 0:
                        self.get_logger().error(
                            'publish on %s failed (%s); retrying'
                            % (topic_names[entry.topic_index], exc)
                        )
                    self.report.retried += 1
                    time.sleep(0.001)

            if published:
                self.report.published += 1
                error_ns = float(released_ns - deadline_ns)
                errors.append(error_ns)
                per_topic_errors[topic_names[entry.topic_index]].append(error_ns)
            else:
                self.report.failed += 1
                self.get_logger().error(
                    'giving up on one message for %s after %d attempts; the run '
                    'continues so the rest of the schedule stays on time'
                    % (topic_names[entry.topic_index], retries + 1)
                )

    # ----------------------------------------------------------------- finish

    def _finish(self) -> None:
        threshold_ms = float(self.get_parameter('fail_on_timing_error_ms').value)
        if threshold_ms > 0.0:
            worst_ms = self.report.overall.max_abs_ns / 1e6
            if worst_ms > threshold_ms:
                self.report.notes.append(
                    'worst publish error %.3f ms exceeded fail_on_timing_error_ms '
                    '%.3f ms' % (worst_ms, threshold_ms)
                )
                self.exit_code = 1

        for line in self.report.format().splitlines():
            self.get_logger().info(line)

        report_path = str(self.get_parameter('report_path').value)
        if report_path:
            try:
                target = Path(report_path).expanduser()
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(self.report.to_json(), encoding='utf-8')
                self.get_logger().info('wrote report %s' % target)
            except OSError as exc:
                self.get_logger().error('could not write report: %s' % exc)

        if not self.report.complete():
            self.exit_code = 1

        self._done.set()

    def wait_until_done(self, timeout: Optional[float] = None) -> bool:
        return self._done.wait(timeout)


def main(args=None) -> int:
    import signal
    import sys

    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node: Optional[ReplayNode] = None
    exit_code = 0
    try:
        node = ReplayNode()
    except SystemExit as exc:
        print('bag_motion_replay: %s' % exc, file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    def handle_interrupt(signum, frame):  # noqa: ARG001 - signal signature
        decision = node.request_stop()
        if decision == 'refused':
            node.get_logger().warning(
                'interrupt ignored: this run publishes the whole recorded '
                'schedule. Press Ctrl-C again within %.1f s to abort and send a '
                'stop command.' % float(node.get_parameter('abort_grace').value)
            )
        else:
            node.get_logger().warning('second interrupt: aborting the run')

    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    worker = threading.Thread(target=node.run_playback, name='replay', daemon=True)
    worker.start()

    try:
        while rclpy.ok() and not node.wait_until_done(0.05):
            rclpy.spin_once(node, timeout_sec=0.0)
        worker.join(timeout=5.0)
        exit_code = node.exit_code
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
