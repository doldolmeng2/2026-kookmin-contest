"""A run recorded with two stacks alive must not be graded as if it were one.

The first sweep left previous launches running and ended up with six main_nodes
publishing to the same topics.  The numbers that came out looked like data - forty
seconds of recording with two and a half thousand mode transitions - so the guard
is on the publish rate, which cannot lie about how many publishers there were.
"""

import pytest

from drive_eval.evaluate import EvaluationReport, Thresholds, _judge


def _report(mode_rate_hz):
    report = EvaluationReport()
    report.mode_rate_hz = mode_rate_hz
    report.playback_rate = 1.0
    report.candidate_rate_hz = 50.0
    report.grading_end_s = 100.0
    return report


def test_a_single_stack_at_the_control_rate_passes():
    report = _report(50.0)

    _judge(report, Thresholds())

    assert report.verdicts['run capture'] == 'PASS'


def test_two_stacks_on_the_same_topics_fail_the_capture():
    report = _report(189.0)

    _judge(report, Thresholds())

    assert report.verdicts['run capture'] == 'FAIL'
    assert any('main_node' in finding for finding in report.findings)


def test_the_capture_failure_names_the_measured_rate():
    report = _report(120.0)

    _judge(report, Thresholds())

    assert '120 Hz' in ' '.join(report.findings)


def test_a_slightly_fast_publisher_is_still_one_stack():
    # Jitter in the 50 Hz timer must not be read as a second node.
    report = _report(55.0)

    _judge(report, Thresholds())

    assert report.verdicts['run capture'] == 'PASS'


def test_the_threshold_is_configurable():
    report = _report(60.0)

    _judge(report, Thresholds(max_mode_rate_hz=55.0))

    assert report.verdicts['run capture'] == 'FAIL'


def test_a_failed_capture_makes_the_whole_report_fail():
    report = _report(189.0)

    _judge(report, Thresholds())

    assert not report.passed()
