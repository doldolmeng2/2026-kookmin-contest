"""Run a replay and grade it in one process, without touching the vehicle topics.

The replay publishes under a prefix (``/selftest`` by default) and the verifier
subscribes to the same prefixed names, so this can be run on the car while the
real stack is up: nothing subscribes to ``/selftest/commands/...``.

Publishing and subscribing inside one process still goes through the middleware —
intra-process delivery is off — so the bytes graded here are the bytes DDS
delivered, not a copy of the in-memory payload.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import List

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from .replay_node import ReplayNode
from .verify_node import VerifyNode

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'


def _overrides(pairs: dict) -> List[Parameter]:
    return [Parameter(name, value=value) for name, value in pairs.items()]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='bag_motion_selftest',
        description='replay a bag onto a private prefix and grade the result',
    )
    parser.add_argument('--bag', default=DEFAULT_BAG)
    parser.add_argument('--cue', default='')
    parser.add_argument('--topic-set', default='actuation')
    parser.add_argument('--include', action='append', default=[])
    parser.add_argument('--exclude', action='append', default=[])
    parser.add_argument('--allow-joycon', action='store_true')
    parser.add_argument('--prefix', default='/selftest')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--start-offset', type=float, default=0.0)
    parser.add_argument(
        '--duration',
        type=float,
        default=-1.0,
        help='seconds of the recording to replay (default: all of it)',
    )
    parser.add_argument('--timing-mode', default='wall', choices=('wall', 'sim'))
    parser.add_argument('--spin-margin-ms', type=float, default=1.5)
    parser.add_argument('--realtime-priority', type=int, default=0)
    parser.add_argument('--idle-timeout', type=float, default=3.0)
    parser.add_argument('--replay-report', default='')
    parser.add_argument('--verify-report', default='')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    rclpy.init(args=None)
    executor = MultiThreadedExecutor(num_threads=4)
    verify = None
    replay = None
    spinner = None

    try:
        end_offset = -1.0 if args.duration < 0.0 else args.start_offset + args.duration
        shared = {
            'bag': args.bag,
            'cue': args.cue,
            'topic_set': args.topic_set,
            'include_topics': args.include or [''],
            'exclude_topics': args.exclude or [''],
            'allow_joycon': bool(args.allow_joycon),
            'topic_prefix': args.prefix,
        }

        verify = VerifyNode(
            parameter_overrides=_overrides(
                dict(
                    shared,
                    idle_timeout=args.idle_timeout,
                    report_path=args.verify_report,
                )
            )
        )
        replay = ReplayNode(
            parameter_overrides=_overrides(
                dict(
                    shared,
                    rate=args.rate,
                    start_offset=args.start_offset,
                    end_offset=end_offset,
                    timing_mode=args.timing_mode,
                    spin_margin_ms=args.spin_margin_ms,
                    realtime_priority=args.realtime_priority,
                    wait_for_subscribers=True,
                    start_delay=0.5,
                    report_path=args.replay_report,
                    safe_stop_on_abort=False,
                )
            )
        )

        executor.add_node(verify)
        executor.add_node(replay)
        spinner = threading.Thread(target=executor.spin, name='selftest-spin', daemon=True)
        spinner.start()

        replay.run_playback()

        deadline = time.monotonic() + args.idle_timeout + 5.0
        while time.monotonic() < deadline and not verify.poll_completion():
            time.sleep(0.05)
        verify.finish()

        print()
        print(replay.report.format())
        print()
        print(verify.report.format())

        exit_code = 0
        if not replay.report.complete():
            exit_code = 1
        if not verify.report.content_exact():
            exit_code = 1
        return exit_code
    finally:
        executor.shutdown(timeout_sec=2.0)
        for node in (replay, verify):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()
        if spinner is not None:
            spinner.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
