"""Fold many evaluation reports into one table.

One bag proves the stack handled one layout.  Tomorrow the cones, the fixed
obstacles and the left-turn timing all move, so the question that matters is
whether the same verdicts hold across every recording available — a scenario that
only passes on the bag it was tuned against has not been covered.

This module is pure: it takes parsed report dictionaries, so the aggregation can
be tested without running anything.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence
import json


VERDICT_ORDER = (
    'run capture',
    'playback',
    'stack output',
    'finishes the course',
    'mode changes',
    'rubbercone',
    'steering',
)

VERDICT_MARKS = {'PASS': '.', 'FAIL': 'X', 'N/A': '-'}


@dataclass
class RunSummary:
    """One graded run, reduced to the columns of the sweep table."""

    name: str = ''
    bag: str = ''
    verdicts: Dict[str, str] = field(default_factory=dict)
    laps_completed: int = 0
    cone_windows: int = 0
    cone_entered: int = 0
    dir_agreement: float = 0.0
    correlation: float = 0.0
    amplitude_ratio: float = 0.0
    dead_output_s: float = 0.0
    graded_s: float = 0.0
    findings: List[str] = field(default_factory=list)

    @classmethod
    def from_report(cls, name: str, report: dict) -> 'RunSummary':
        steering = report.get('steering_driving') or report.get('steering_overall') or {}
        if not steering.get('samples'):
            steering = report.get('steering_overall') or {}
        sessions = report.get('cone_sessions') or {}
        return cls(
            name=name,
            bag=str(report.get('source_bag', '')).rsplit('/', 1)[-1],
            verdicts=dict(report.get('verdicts') or {}),
            laps_completed=int(report.get('laps_completed', 0)),
            cone_windows=int(sessions.get('windows', 0)),
            cone_entered=int(sessions.get('entered', 0)),
            dir_agreement=float(steering.get('sign_agreement', 0.0)),
            correlation=float(steering.get('correlation', 0.0)),
            amplitude_ratio=float(steering.get('amplitude_ratio', 0.0)),
            dead_output_s=float(report.get('dead_output_s', 0.0)),
            graded_s=float(report.get('grading_end_s', 0.0))
            - float(report.get('overlap_start_s', 0.0)),
            findings=list(report.get('findings') or []),
        )

    def passed(self) -> bool:
        return bool(self.verdicts) and all(
            value != 'FAIL' for value in self.verdicts.values()
        )


def load_summaries(report_paths: Sequence) -> List[RunSummary]:
    """Read ``report.json`` files, skipping any that cannot be parsed."""
    summaries: List[RunSummary] = []
    for path in report_paths:
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                report = json.load(handle)
        except (OSError, ValueError):
            continue
        name = getattr(path, 'parent', None)
        summaries.append(
            RunSummary.from_report(name.name if name else str(path), report)
        )
    return summaries


def verdict_columns(summaries: Iterable[RunSummary]) -> List[str]:
    """Verdict names in a stable order, with any unexpected ones appended."""
    seen = set()
    for summary in summaries:
        seen.update(summary.verdicts)
    ordered = [name for name in VERDICT_ORDER if name in seen]
    ordered.extend(sorted(name for name in seen if name not in VERDICT_ORDER))
    return ordered


def format_matrix(summaries: Sequence[RunSummary]) -> str:
    """One row per run, one column per verdict, plus the numbers behind them."""
    if not summaries:
        return 'no evaluation reports found'

    columns = verdict_columns(summaries)
    lines = ['legend: . pass   X fail   - not exercised', '']
    header = '%-38s %-6s' % ('run', 'laps')
    header += ''.join('%-4s' % name[:3] for name in columns)
    header += '  %-9s %-7s %-6s %-7s %s' % (
        'cones', 'dir', 'r', 'amp', 'graded'
    )
    lines.append(header)
    lines.append('-' * len(header))

    for summary in summaries:
        row = '%-38s %-6s' % (summary.name[:38], '%d/3' % summary.laps_completed)
        for name in columns:
            row += '%-4s' % VERDICT_MARKS.get(summary.verdicts.get(name, 'N/A'), '?')
        row += '  %-9s %5.0f%%  %+.2f  %5.2f  %6.1fs' % (
            '%d/%d' % (summary.cone_entered, summary.cone_windows)
            if summary.cone_windows
            else '-',
            100.0 * summary.dir_agreement,
            summary.correlation,
            summary.amplitude_ratio,
            summary.graded_s,
        )
        lines.append(row)

    lines.append('-' * len(header))
    lines.append('columns: ' + ', '.join('%s=%s' % (name[:3], name) for name in columns))
    return '\n'.join(lines)


def format_findings(summaries: Sequence[RunSummary]) -> str:
    """Every finding, grouped by run, so a sweep does not hide the detail."""
    blocks: List[str] = []
    for summary in summaries:
        if not summary.findings:
            continue
        blocks.append('%s:' % summary.name)
        for finding in summary.findings:
            blocks.append('  - %s' % finding)
    if not blocks:
        return 'no findings in any run'
    return '\n'.join(blocks)


def recurring_findings(summaries: Sequence[RunSummary]) -> List[str]:
    """Findings that showed up in more than one recording.

    A problem that survives a change of layout is a problem with the code, not
    with one day's cone placement — those are the ones worth fixing first.
    """
    tally: Dict[str, int] = {}
    for summary in summaries:
        for finding in {_generalize(text) for text in summary.findings}:
            tally[finding] = tally.get(finding, 0) + 1
    return [
        '%s  (in %d of %d runs)' % (text, count, len(summaries))
        for text, count in sorted(tally.items(), key=lambda item: -item[1])
        if count > 1
    ]


def _generalize(finding: str) -> str:
    """Strip the numbers so the same problem in two runs matches."""
    out = []
    in_number = False
    for character in finding:
        if character.isdigit() or (character == '.' and in_number):
            if not in_number:
                out.append('N')
                in_number = True
            continue
        in_number = False
        out.append(character)
    return ''.join(out)


def overall(summaries: Sequence[RunSummary]) -> str:
    passed = sum(1 for summary in summaries if summary.passed())
    return '%d of %d runs passed every verdict' % (passed, len(summaries))
