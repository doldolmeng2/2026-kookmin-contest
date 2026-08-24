"""The sweep exists to separate 'broken on this layout' from 'broken'."""

from drive_eval.summary import (
    RunSummary,
    format_matrix,
    overall,
    recurring_findings,
    verdict_columns,
)


def _report(**overrides):
    report = {
        'source_bag': '/home/x/my_rosbag/rosbag2_2026_08_13-13_33_23',
        'verdicts': {'steering': 'PASS', 'rubbercone': 'PASS'},
        'laps_completed': 3,
        'grading_end_s': 100.0,
        'overlap_start_s': 0.0,
        'dead_output_s': 0.0,
        'cone_sessions': {'windows': 2, 'entered': 2},
        'steering_driving': {
            'samples': 100,
            'sign_agreement': 0.8,
            'correlation': 0.4,
            'amplitude_ratio': 0.6,
        },
        'findings': [],
    }
    report.update(overrides)
    return report


def test_a_run_with_no_failing_verdict_passes():
    assert RunSummary.from_report('run1', _report()).passed()


def test_one_failing_verdict_fails_the_run():
    summary = RunSummary.from_report(
        'run1', _report(verdicts={'steering': 'FAIL', 'rubbercone': 'PASS'})
    )

    assert not summary.passed()


def test_a_scenario_that_never_exercised_cones_is_not_counted_as_a_failure():
    summary = RunSummary.from_report(
        'run1', _report(verdicts={'steering': 'PASS', 'rubbercone': 'N/A'})
    )

    assert summary.passed()


def test_steering_falls_back_to_the_whole_run_when_nothing_was_dead():
    report = _report()
    report['steering_driving'] = {'samples': 0}
    report['steering_overall'] = {
        'samples': 50,
        'sign_agreement': 0.6,
        'correlation': 0.3,
        'amplitude_ratio': 0.5,
    }

    summary = RunSummary.from_report('run1', report)

    assert summary.dir_agreement == 0.6


def test_a_finding_seen_in_two_recordings_is_flagged_as_recurring():
    # Same defect, different numbers: a code problem, not a layout problem.
    first = RunSummary.from_report(
        'a', _report(findings=['corridor #1 (10.00-18.00 s) never entered CONE_DRIVE'])
    )
    second = RunSummary.from_report(
        'b', _report(findings=['corridor #2 (44.50-51.25 s) never entered CONE_DRIVE'])
    )

    recurring = recurring_findings([first, second])

    assert len(recurring) == 1
    assert 'never entered CONE_DRIVE' in recurring[0]
    assert 'in 2 of 2 runs' in recurring[0]


def test_a_finding_seen_once_is_not_flagged_as_recurring():
    first = RunSummary.from_report('a', _report(findings=['something odd at 3.0 s']))
    second = RunSummary.from_report('b', _report())

    assert recurring_findings([first, second]) == []


def test_verdict_columns_keep_a_stable_order_and_keep_unknown_ones():
    summaries = [
        RunSummary.from_report(
            'a', _report(verdicts={'steering': 'PASS', 'brand new': 'PASS'})
        )
    ]

    assert verdict_columns(summaries) == ['steering', 'brand new']


def test_matrix_renders_a_row_per_run():
    summaries = [
        RunSummary.from_report('alpha', _report()),
        RunSummary.from_report('beta', _report(verdicts={'steering': 'FAIL'})),
    ]

    text = format_matrix(summaries)

    assert 'alpha' in text and 'beta' in text
    assert 'X' in text


def test_overall_counts_the_passing_runs():
    summaries = [
        RunSummary.from_report('a', _report()),
        RunSummary.from_report('b', _report(verdicts={'steering': 'FAIL'})),
    ]

    assert overall(summaries) == '1 of 2 runs passed every verdict'


def test_an_empty_sweep_says_so_instead_of_crashing():
    assert 'no evaluation reports' in format_matrix([])
