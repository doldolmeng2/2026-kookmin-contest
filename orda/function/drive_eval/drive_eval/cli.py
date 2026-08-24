"""Command line for drive_eval."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .bagsource import BagReadError, load_reference
from .cone_truth import ConeDetectorConfig, cone_windows, describe_windows, observe
from .evaluate import (
    REFERENCE_SCAN_TOPIC,
    RUN_MODE_TOPIC,
    RUN_STEERING_TOPIC,
    Thresholds,
    bag_start_ns,
    evaluate,
    scan_samples,
)
from .summary import (
    format_findings,
    format_matrix,
    load_summaries,
    overall,
    recurring_findings,
)
from .timebase import TimebaseError

DEFAULT_SOURCE_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'


def _cone_config(args) -> ConeDetectorConfig:
    return ConeDetectorConfig(
        max_range_m=args.cone_max_range,
        max_angle_deg=args.cone_max_angle,
        max_lateral_m=args.cone_max_lateral,
    )


def _cone_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--cone-max-range', type=float, default=1.10)
    parser.add_argument('--cone-max-angle', type=float, default=85.0)
    parser.add_argument('--cone-max-lateral', type=float, default=0.85)


def cmd_cones(args) -> int:
    source = Path(args.source_bag).expanduser()
    reference = load_reference(
        source, (REFERENCE_SCAN_TOPIC,), use_cache=not args.no_cache, log=print
    )
    scans = reference[REFERENCE_SCAN_TOPIC]
    if not scans:
        print('no /scan messages in %s' % source, file=sys.stderr)
        return 1

    samples = scan_samples(scans, bag_start_ns(source, scans[0][0]))
    windows = cone_windows([observe(sample, _cone_config(args)) for sample in samples])
    print()
    print('cone corridors in %s:' % source.name)
    print(describe_windows(windows))
    return 0


def cmd_evaluate(args) -> int:
    thresholds = Thresholds()
    try:
        report = evaluate(
            source_bag=args.source_bag,
            run_bag=args.run_bag,
            steering_topic=args.steering_topic,
            mode_topic=args.mode_topic,
            thresholds=thresholds,
            cone_config=_cone_config(args),
            grid_hz=args.grid_hz,
            use_cache=not args.no_cache,
            log=print,
        )
    except (BagReadError, TimebaseError) as exc:
        print('drive_eval: %s' % exc, file=sys.stderr)
        return 2

    print()
    print(report.format())

    if args.json:
        target = Path(args.json).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(report.to_json(), encoding='utf-8')
        print('wrote %s' % target)

    return 0 if report.passed() else 1


def cmd_summary(args) -> int:
    root = Path(args.runs).expanduser()
    if not root.is_dir():
        print('no such directory: %s' % root, file=sys.stderr)
        return 2

    reports = sorted(root.glob('*/report.json'))
    if not reports:
        reports = sorted(root.glob('report.json'))
    summaries = load_summaries(reports)
    if not summaries:
        print('no report.json under %s' % root, file=sys.stderr)
        return 2

    print('=' * 78)
    print('drive_eval sweep - %d run(s) under %s' % (len(summaries), root))
    print('=' * 78)
    print(format_matrix(summaries))
    print()
    recurring = recurring_findings(summaries)
    if recurring:
        print('findings that survived a change of recording (fix these first):')
        for finding in recurring:
            print('  - %s' % finding)
        print()
    print(format_findings(summaries))
    print()
    print('  %s' % overall(summaries))
    print('=' * 78)
    return 0 if all(summary.passed() for summary in summaries) else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='drive_eval',
        description='grade an xycar_ws stack run against the bag that drove it',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    cones = sub.add_parser(
        'cones', help='show the cone corridors /scan contains'
    )
    cones.add_argument('--source-bag', default=DEFAULT_SOURCE_BAG)
    cones.add_argument('--no-cache', action='store_true')
    _cone_args(cones)
    cones.set_defaults(func=cmd_cones)

    run = sub.add_parser('evaluate', help='grade a recorded run')
    run.add_argument('--source-bag', default=DEFAULT_SOURCE_BAG)
    run.add_argument('--run-bag', required=True)
    run.add_argument('--steering-topic', default=RUN_STEERING_TOPIC)
    run.add_argument('--mode-topic', default=RUN_MODE_TOPIC)
    run.add_argument('--grid-hz', type=float, default=20.0)
    run.add_argument('--json', default='')
    run.add_argument('--no-cache', action='store_true')
    _cone_args(run)
    run.set_defaults(func=cmd_evaluate)

    sweep = sub.add_parser(
        'summary', help='aggregate every report.json under a directory'
    )
    sweep.add_argument('--runs', default='~/drive_eval_runs')
    sweep.set_defaults(func=cmd_summary)

    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
