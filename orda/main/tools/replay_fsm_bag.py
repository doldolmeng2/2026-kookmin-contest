#!/usr/bin/env python3
"""Command-line entry point for read-only FSM replay over a rosbag2 directory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Optional, Sequence


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from main.bag_replay import (  # noqa: E402
    BagEnvironmentError,
    OfflineBagReplay,
    SequentialBagReader,
    write_report_files,
)
from main.race_fsm import Mode  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay timestamped bag records through the pure FSM",
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument(
        "--start-mode",
        choices=[mode.value for mode in Mode],
        default=Mode.WAIT_GREEN.value,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/xytron/fsm_replay_reports"),
    )
    parser.add_argument("--max-events", type=int)
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print only the compact result summary; report files remain complete",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_events is not None and args.max_events < 1:
        raise SystemExit("--max-events must be positive")

    reader = SequentialBagReader(args.bag)
    try:
        reader.open()
        replay = OfflineBagReplay(start_mode=Mode(args.start_mode))
        report = replay.run(
            reader.iter_events(args.max_events),
            bag_path=reader.bag_path,
            topic_types=reader.topic_types,
        )
    except (BagEnvironmentError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    json_path, text_path = write_report_files(report, args.output_dir)
    summary = {
        "bag": report["basic_info"]["bag_path"],
        "start_mode": report["fsm"]["initial_mode"],
        "final_mode": report["fsm"]["final_mode"],
        "transitions": report["fsm"]["transition_count"],
        "invariant_pass": report["validation"]["invariant_pass"],
        "warnings": len(report["validation"]["warnings"]),
        "json_report": str(json_path),
        "text_report": str(text_path),
    }
    if args.summary_only:
        print(json.dumps(summary, ensure_ascii=False))
    else:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
