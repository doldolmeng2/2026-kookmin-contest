"""Check a replay against the recording it came from.

Subscribes to the same topics the replay publishes, in raw mode so the CDR bytes
arrive untouched, and compares them with the cue:

* every payload, in order, byte for byte;
* the gap between consecutive messages, against the recorded gap;
* each message's position on the timeline, against its recorded position.

Receive timestamps include DDS transport latency, which is not replay error — but
it applies to every message alike, so it cancels out of both the interval and the
offset comparison.  That is why the report never claims an absolute latency.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional
import threading
import time

import rclpy
from rclpy.node import Node

from .bagio import BagError, open_bag
from .cue import CueData, CueError, load_or_build_cue
from .qos import merge_offered_qos, parse_offered_qos, to_qos_profile
from .report import TimingStats, VerificationReport
from .topic_sets import TopicSelectionError, resolve_selection

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'


class VerifyNode(Node):
    """Record what a replay puts on the wire and grade it against the bag."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__('bag_motion_verify', **node_kwargs)

        self.declare_parameter('bag', DEFAULT_BAG)
        self.declare_parameter('cue', '')
        self.declare_parameter('topic_set', 'actuation')
        self.declare_parameter('include_topics', [''])
        self.declare_parameter('exclude_topics', [''])
        self.declare_parameter('allow_joycon', False)
        self.declare_parameter('topic_prefix', '')
        self.declare_parameter('idle_timeout', 5.0)
        self.declare_parameter('max_duration', 0.0)
        self.declare_parameter('report_path', '')
        self.declare_parameter('shutdown_when_done', True)

        self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        self.cue: Optional[CueData] = None
        self.received: Dict[str, List[tuple]] = {}
        self._lock = threading.Lock()
        self._last_message_at: Optional[float] = None
        self._started_at = time.monotonic()
        self._done = threading.Event()
        self.report = VerificationReport()
        self.exit_code = 0

        self._prepare()

    def _string_list(self, name: str) -> List[str]:
        raw = self.get_parameter(name).value or []
        return [str(item) for item in raw if str(item).strip()]

    def _prepare(self) -> None:
        log = self.get_logger()
        bag_path = str(self.get_parameter('bag').value)

        try:
            available = sorted(open_bag(bag_path).topics())
        except BagError as exc:
            raise SystemExit('cannot open bag: %s' % exc) from exc

        try:
            selection = resolve_selection(
                available=available,
                topic_set=str(self.get_parameter('topic_set').value),
                include_topics=self._string_list('include_topics'),
                exclude_topics=self._string_list('exclude_topics'),
                allow_joycon=bool(self.get_parameter('allow_joycon').value),
            )
        except TopicSelectionError as exc:
            raise SystemExit('topic selection failed: %s' % exc) from exc

        try:
            cue, _ = load_or_build_cue(
                bag_path=bag_path,
                topic_names=selection.topics,
                cue_path=str(self.get_parameter('cue').value) or None,
                log=log.info,
            )
        except (BagError, CueError) as exc:
            raise SystemExit('cannot load replay cue: %s' % exc) from exc

        self.cue = cue
        self.received = {topic.name: [] for topic in cue.topics}

        from rosidl_runtime_py.utilities import get_message

        for topic in cue.topics:
            spec = merge_offered_qos(parse_offered_qos(topic.offered_qos_profiles))
            profile = to_qos_profile(spec)
            name = self.topic_prefix + topic.name if self.topic_prefix else topic.name
            self.create_subscription(
                get_message(topic.type_name),
                name,
                self._make_callback(topic.name),
                profile,
                raw=True,
            )
            log.info('watching %s' % name)

        log.info(
            'waiting for the replay; the run ends %.1f s after the last message'
            % float(self.get_parameter('idle_timeout').value)
        )

    def _make_callback(self, topic_name: str):
        def callback(payload: bytes) -> None:
            now = time.monotonic_ns()
            with self._lock:
                self.received[topic_name].append((now, bytes(payload)))
                self._last_message_at = time.monotonic()

        return callback

    def poll_completion(self) -> bool:
        """True once the replay has clearly stopped sending."""
        if self._done.is_set():
            return True
        idle_timeout = float(self.get_parameter('idle_timeout').value)
        max_duration = float(self.get_parameter('max_duration').value)
        now = time.monotonic()
        if max_duration > 0.0 and now - self._started_at >= max_duration:
            self.get_logger().warning(
                'max_duration reached; grading what arrived so far'
            )
            self.finish()
            return True
        with self._lock:
            last = self._last_message_at
        if last is not None and now - last >= idle_timeout:
            self.finish()
            return True
        return False

    def finish(self) -> None:
        if self._done.is_set():
            return
        with self._lock:
            snapshot = {name: list(items) for name, items in self.received.items()}
        self.report = self._grade(snapshot)
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

        if not self.report.content_exact():
            self.exit_code = 1
        self._done.set()

    def _grade(self, snapshot: Dict[str, List[tuple]]) -> VerificationReport:
        report = VerificationReport(topics=list(self.cue.topic_names()))
        all_interval_errors: List[float] = []
        all_offset_errors: List[float] = []

        for topic in report.topics:
            expected = self.cue.records_for(topic)
            actual = snapshot.get(topic, [])
            report.expected[topic] = len(expected)
            report.received[topic] = len(actual)

            mismatches = 0
            for index in range(min(len(expected), len(actual))):
                if expected[index][1] != actual[index][1]:
                    mismatches += 1
                    if report.first_mismatch is None:
                        report.first_mismatch = (
                            '%s message %d: recorded %d bytes, received %d bytes'
                            % (
                                topic,
                                index,
                                len(expected[index][1]),
                                len(actual[index][1]),
                            )
                        )
            report.payload_mismatches[topic] = mismatches

            if len(actual) != len(expected):
                report.notes.append(
                    '%s: received %d of %d recorded messages'
                    % (topic, len(actual), len(expected))
                )

            usable = min(len(expected), len(actual))
            if usable >= 2:
                interval_errors = [
                    float(
                        (actual[i][0] - actual[i - 1][0])
                        - (expected[i][0] - expected[i - 1][0])
                    )
                    for i in range(1, usable)
                ]
                offset_errors = [
                    float(
                        (actual[i][0] - actual[0][0])
                        - (expected[i][0] - expected[0][0])
                    )
                    for i in range(1, usable)
                ]
                report.interval_stats[topic] = TimingStats.from_errors(interval_errors)
                report.offset_stats[topic] = TimingStats.from_errors(offset_errors)
                all_interval_errors.extend(interval_errors)
                all_offset_errors.extend(offset_errors)

        report.overall_interval = TimingStats.from_errors(all_interval_errors)
        report.overall_offset = TimingStats.from_errors(all_offset_errors)
        return report

    def wait_until_done(self, timeout: Optional[float] = None) -> bool:
        return self._done.wait(timeout)


def main(args=None) -> int:
    import sys

    rclpy.init(args=args)
    node: Optional[VerifyNode] = None
    try:
        node = VerifyNode()
    except SystemExit as exc:
        print('bag_motion_verify: %s' % exc, file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    exit_code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.poll_completion():
                break
        exit_code = node.exit_code
    except KeyboardInterrupt:
        node.finish()
        exit_code = node.exit_code
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
