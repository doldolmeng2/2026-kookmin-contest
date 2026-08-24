"""Compare the steering the stack produced with the steering that was recorded.

The reference is a human driving with a stick, so an exact match is not the bar
and a small RMSE is not the goal — a driver and a controller can take the same
corner through different lines.  What the numbers are asked to show is whether the
stack steers *the same way*: same direction at the same moment, comparable
amplitude, and no systematic lag.

So the report leads with sign agreement and correlation, keeps RMSE as context,
and estimates lag by cross-correlation, because "steers correctly but 300 ms late"
and "steers wrongly" have very different fixes and identical RMSE.

Pure module: takes plain series, returns plain numbers.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple
import math

from .timebase import Series


@dataclass(frozen=True)
class SteeringStats:
    """How the candidate steering relates to the recorded one."""

    samples: int = 0
    active_samples: int = 0
    correlation: float = 0.0
    sign_agreement: float = 0.0
    rmse_deg: float = 0.0
    mae_deg: float = 0.0
    bias_deg: float = 0.0
    max_abs_error_deg: float = 0.0
    reference_rms_deg: float = 0.0
    candidate_rms_deg: float = 0.0
    amplitude_ratio: float = 0.0
    best_lag_s: float = 0.0
    correlation_at_best_lag: float = 0.0
    # RMS 비만으로는 조향 크기를 못 읽는다. 이 bag 의 사람 조향은 라바콘 구간에서
    # 중앙값 1.5°, 표본의 27~33% 가 풀락 100° 인 이봉분포라 RMS 가 스파이크에
    # 끌려간다. 같은 구간에서 스택은 최대 44°, 평균 19° 로 계속 꺾고 있는데도
    # 비율은 0.31 로 나와 "거의 안 꺾는다" 처럼 보였다 — 실제로는 45° 제한에
    # 닿지도 않았다. 그래서 분위수와 최대값을 따로 남긴다.
    reference_median_abs_deg: float = 0.0
    candidate_median_abs_deg: float = 0.0
    reference_p90_abs_deg: float = 0.0
    candidate_p90_abs_deg: float = 0.0
    reference_max_abs_deg: float = 0.0
    candidate_max_abs_deg: float = 0.0

    def as_dict(self) -> Dict[str, float]:
        return asdict(self)

    def format(self, label: str) -> str:
        if not self.samples:
            return '  %-28s (no overlapping samples)' % label
        return (
            '  %-28s n=%5d  dir-agree=%5.1f%%  r=%+.3f  lag=%+.2fs (r=%+.3f)\n'
            '  %-28s rmse=%6.2f deg  bias=%+6.2f deg  amp(cand/ref)=%.2f\n'
            '  %-28s |angle| med %5.1f/%5.1f  p90 %5.1f/%5.1f  max %5.1f/%5.1f '
            '(stack/recorded)'
            % (
                label,
                self.samples,
                100.0 * self.sign_agreement,
                self.correlation,
                self.best_lag_s,
                self.correlation_at_best_lag,
                '',
                self.rmse_deg,
                self.bias_deg,
                self.amplitude_ratio,
                '',
                self.candidate_median_abs_deg,
                self.reference_median_abs_deg,
                self.candidate_p90_abs_deg,
                self.reference_p90_abs_deg,
                self.candidate_max_abs_deg,
                self.reference_max_abs_deg,
            )
        )


def build_grid(start_s: float, end_s: float, hz: float) -> List[float]:
    """Uniform comparison grid; both series are held forward onto it."""
    if hz <= 0.0:
        raise ValueError('grid hz must be positive')
    if end_s <= start_s:
        return []
    step = 1.0 / hz
    count = int(math.floor((end_s - start_s) / step)) + 1
    return [start_s + index * step for index in range(count)]


def hold_onto(series: Series, grid: Sequence[float]) -> List[Optional[float]]:
    """Zero-order hold: the value a subscriber would have had at each grid point."""
    return [series.value_at(time_s) for time_s in grid]


def _pearson(left: Sequence[float], right: Sequence[float]) -> float:
    count = len(left)
    if count < 2:
        return 0.0
    mean_left = sum(left) / count
    mean_right = sum(right) / count
    numerator = sum(
        (a - mean_left) * (b - mean_right) for a, b in zip(left, right)
    )
    left_var = sum((a - mean_left) ** 2 for a in left)
    right_var = sum((b - mean_right) ** 2 for b in right)
    if left_var <= 0.0 or right_var <= 0.0:
        return 0.0
    return numerator / math.sqrt(left_var * right_var)


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    """Linear-interpolated percentile of an already sorted sequence."""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = fraction * (len(sorted_values) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(sorted_values) - 1)
    weight = position - low
    return float(sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight)


def _rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _shift_correlation(
    reference: Sequence[float], candidate: Sequence[float], shift: int
) -> float:
    """Correlation with the candidate moved ``shift`` grid steps earlier/later."""
    if shift >= 0:
        left = reference[: len(reference) - shift] if shift else reference
        right = candidate[shift:]
    else:
        left = reference[-shift:]
        right = candidate[: len(candidate) + shift]
    if len(left) < 8:
        return 0.0
    return _pearson(list(left), list(right))


def compare_steering(
    reference: Series,
    candidate: Series,
    start_s: float,
    end_s: float,
    grid_hz: float = 20.0,
    active_threshold_deg: float = 5.0,
    max_lag_s: float = 1.5,
) -> SteeringStats:
    """Grade ``candidate`` steering against ``reference`` over one window.

    ``active_threshold_deg`` keeps the direction-agreement figure meaningful: on a
    straight both series sit near zero, and counting sign matches there would
    report agreement for noise.
    """
    grid = build_grid(start_s, end_s, grid_hz)
    if not grid:
        return SteeringStats()

    reference_held = hold_onto(reference, grid)
    candidate_held = hold_onto(candidate, grid)

    pairs = [
        (a, b)
        for a, b in zip(reference_held, candidate_held)
        if a is not None and b is not None
    ]
    if len(pairs) < 2:
        return SteeringStats()

    reference_values = [pair[0] for pair in pairs]
    candidate_values = [pair[1] for pair in pairs]
    errors = [b - a for a, b in pairs]

    active = [
        (a, b) for a, b in pairs if abs(a) >= active_threshold_deg
    ]
    agreeing = sum(
        1
        for a, b in active
        if (a > 0 and b > 0) or (a < 0 and b < 0)
    )

    reference_rms = _rms(reference_values)
    candidate_rms = _rms(candidate_values)
    reference_abs = sorted(abs(value) for value in reference_values)
    candidate_abs = sorted(abs(value) for value in candidate_values)

    max_shift = max(1, int(round(max_lag_s * grid_hz)))
    best_shift = 0
    best_correlation = _pearson(reference_values, candidate_values)
    for shift in range(-max_shift, max_shift + 1):
        value = _shift_correlation(reference_values, candidate_values, shift)
        if value > best_correlation:
            best_correlation = value
            best_shift = shift

    return SteeringStats(
        samples=len(pairs),
        active_samples=len(active),
        correlation=_pearson(reference_values, candidate_values),
        sign_agreement=(agreeing / len(active)) if active else 0.0,
        rmse_deg=_rms(errors),
        mae_deg=sum(abs(error) for error in errors) / len(errors),
        bias_deg=sum(errors) / len(errors),
        max_abs_error_deg=max(abs(error) for error in errors),
        reference_rms_deg=reference_rms,
        candidate_rms_deg=candidate_rms,
        amplitude_ratio=(candidate_rms / reference_rms) if reference_rms else 0.0,
        best_lag_s=best_shift / grid_hz,
        correlation_at_best_lag=best_correlation,
        reference_median_abs_deg=_percentile(reference_abs, 0.5),
        candidate_median_abs_deg=_percentile(candidate_abs, 0.5),
        reference_p90_abs_deg=_percentile(reference_abs, 0.9),
        candidate_p90_abs_deg=_percentile(candidate_abs, 0.9),
        reference_max_abs_deg=reference_abs[-1],
        candidate_max_abs_deg=candidate_abs[-1],
    )


def saturation_ratio(series: Series, limit_deg: float = 99.0) -> float:
    """Fraction of commands pinned at the steering limit.

    A controller that spends its time at the stop is not tracking anything, so
    this guards against a high correlation that is really two square waves.
    """
    if not len(series):
        return 0.0
    pinned = sum(1 for value in series.values if abs(value) >= limit_deg)
    return pinned / len(series)


def zero_output_spans(
    series: Series,
    tolerance_deg: float = 0.05,
    min_duration_s: float = 2.0,
) -> List[Tuple[float, float]]:
    """Stretches where the command was flat zero for a meaningful time.

    A node that keeps publishing zeros looks alive to every liveness check and to
    the publish-rate figure, while the car does nothing.  Averaged over a whole
    run those zeros also quietly drag the correlation down, so they have to be
    named rather than absorbed into a single number.
    """
    spans: List[Tuple[float, float]] = []
    start: Optional[float] = None
    previous_time: Optional[float] = None

    for time_s, value in zip(series.times_s, series.values):
        if abs(value) <= tolerance_deg:
            if start is None:
                start = time_s
        elif start is not None:
            if previous_time is not None and previous_time - start >= min_duration_s:
                spans.append((start, previous_time))
            start = None
        previous_time = time_s

    if start is not None and previous_time is not None:
        if previous_time - start >= min_duration_s:
            spans.append((start, previous_time))
    return spans


def command_rate_hz(series: Series) -> float:
    """Mean publish rate of a command series, for a liveness check."""
    if len(series) < 2:
        return 0.0
    span = series.times_s[-1] - series.times_s[0]
    if span <= 0.0:
        return 0.0
    return (len(series) - 1) / span


def windowed_comparison(
    reference: Series,
    candidate: Series,
    windows: Sequence[Tuple[str, float, float]],
    grid_hz: float = 20.0,
) -> Dict[str, SteeringStats]:
    """Compare over several named windows, e.g. one per cone section."""
    return {
        name: compare_steering(reference, candidate, start_s, end_s, grid_hz=grid_hz)
        for name, start_s, end_s in windows
    }
