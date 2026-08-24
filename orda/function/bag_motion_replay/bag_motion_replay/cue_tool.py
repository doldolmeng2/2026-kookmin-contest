"""Command line access to the cue: inspect a bag, build a cue, describe a cue.

Building the cue is the only step that touches the 21 GB recording, so it is worth
doing once, on purpose, before a run rather than implicitly at launch time.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .bagio import BagError, open_bag
from .cue import (
    CueError,
    collect_cue,
    default_cue_path,
    read_cue,
    summarize_cue,
    write_cue,
)
from .topic_sets import TopicSelectionError, describe_selection, resolve_selection

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'


def _selection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--topic-set', default='actuation')
    parser.add_argument('--include', action='append', default=[])
    parser.add_argument('--exclude', action='append', default=[])
    parser.add_argument(
        '--allow-joycon',
        action='store_true',
        help='also replay /joy and /xycar_motor (refused by default: those carry '
             'the steering and speed the stick produced)',
    )


def _resolve(args, available):
    return resolve_selection(
        available=available,
        topic_set=args.topic_set,
        include_topics=args.include,
        exclude_topics=args.exclude,
        allow_joycon=args.allow_joycon,
    )


def cmd_info(args) -> int:
    source = open_bag(args.bag)
    topics = source.topics()
    print('bag: %s' % Path(args.bag).expanduser())
    print('%-36s %-40s %8s' % ('topic', 'type', 'messages'))
    for name in sorted(topics):
        topic = topics[name]
        print('%-36s %-40s %8d' % (name, topic.type_name, topic.count))
    print()
    try:
        selection = _resolve(args, sorted(topics))
    except TopicSelectionError as exc:
        print('topic selection: %s' % exc)
        return 1
    print(describe_selection(selection))
    return 0


def cmd_build(args) -> int:
    source = open_bag(args.bag)
    selection = _resolve(args, sorted(source.topics()))
    print(describe_selection(selection))
    print()

    def progress(done: int, expected: int) -> None:
        if expected:
            sys.stderr.write('\r  reading %d / ~%d records' % (done, expected))
            sys.stderr.flush()

    cue = collect_cue(args.bag, selection.topics, progress=progress)
    sys.stderr.write('\r' + ' ' * 60 + '\r')
    target = Path(args.out).expanduser() if args.out else default_cue_path(
        args.bag, selection.topics
    )
    write_cue(cue, target)
    print(summarize_cue(cue))
    print()
    print('cue written to %s' % target)
    return 0


def cmd_show(args) -> int:
    cue = read_cue(args.cue)
    print(summarize_cue(cue))
    print()
    print('built from %s' % cue.source.get('bag_uri', '(unknown)'))
    print('built at   %s' % cue.source.get('built_at', '(unknown)'))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='bag_motion_cue',
        description='inspect a rosbag2 and prepare a replay cue',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    info = sub.add_parser('info', help='list the topics a bag holds')
    info.add_argument('bag', nargs='?', default=DEFAULT_BAG)
    _selection_args(info)
    info.set_defaults(func=cmd_info)

    build = sub.add_parser('build', help='extract the replay cue from a bag')
    build.add_argument('bag', nargs='?', default=DEFAULT_BAG)
    build.add_argument('--out', default='')
    _selection_args(build)
    build.set_defaults(func=cmd_build)

    show = sub.add_parser('show', help='describe an existing cue file')
    show.add_argument('cue')
    show.set_defaults(func=cmd_show)

    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (BagError, CueError, TopicSelectionError) as exc:
        print('bag_motion_cue: %s' % exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
