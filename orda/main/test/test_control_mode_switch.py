"""모드 전환 첫 사이클의 조향.

2026-08-13 bag 을 스택에 물려 돌렸을 때, LANE_DRIVE → FIXED_AVOID 전환 순간
(28.98 s) 에 -135.83° 가 한 번 나갔다. 다음 사이클부터는 -34.7° 근처였으니
오프셋이 컸던 게 아니라 미분항이 통째로 튄 것이다 — reset() 이 prev_offset 을
0 으로 되돌려 diff 가 오프셋 전체가 됐다. FIXED_AVOID 는 kp 0.12 · kd 0.35 라
그 한 사이클만 0.47·e 가 된다.

내일은 고정장애물 위치가 달라진다. 킥의 크기는 그 순간 오프셋에 비례하므로
배치가 바뀌면 크기도 바뀐다.
"""

import pytest

from main.control import Controller
from main.race_fsm import Mode


LANE_LIMIT = 100.0
CONE_LIMIT = 45.0


def controller():
    return Controller()


def test_the_first_cycle_of_a_new_mode_has_no_derivative_kick():
    control = controller()
    control.update(Mode.LANE_DRIVE, 0, float('inf'))

    control.update(Mode.FIXED_AVOID, -289, float('inf'))

    # kp only: 0.12 * -289 = -34.68. With the old kick it was 0.47 * -289.
    assert control.get_angle() == pytest.approx(-34.68, abs=0.01)


def test_the_derivative_term_still_works_from_the_second_cycle():
    control = controller()
    control.update(Mode.FIXED_AVOID, -100, float('inf'))

    control.update(Mode.FIXED_AVOID, -200, float('inf'))

    # 0.12 * -200 + 0.35 * (-200 - -100) = -24 - 35
    assert control.get_angle() == pytest.approx(-59.0, abs=0.01)


def test_a_steady_offset_produces_no_derivative_contribution():
    control = controller()
    control.update(Mode.FIXED_AVOID, -150, float('inf'))

    control.update(Mode.FIXED_AVOID, -150, float('inf'))

    assert control.get_angle() == pytest.approx(0.12 * -150, abs=0.01)


def test_switching_away_and_back_re_arms_the_unknown_previous_offset():
    control = controller()
    control.update(Mode.FIXED_AVOID, -100, float('inf'))
    control.update(Mode.FIXED_AVOID, -200, float('inf'))
    control.update(Mode.LANE_DRIVE, 0, float('inf'))

    control.update(Mode.FIXED_AVOID, -289, float('inf'))

    assert control.get_angle() == pytest.approx(-34.68, abs=0.01)


def test_fixed_avoid_output_is_bounded_like_lane_driving():
    # 다른 모드는 모두 상한이 있는데 FIXED_AVOID 만 없었다.
    control = controller()
    control.update(Mode.FIXED_AVOID, -5000, float('inf'))

    assert abs(control.get_angle()) <= LANE_LIMIT + 1e-6


def test_fixed_avoid_bound_applies_to_both_directions():
    control = controller()
    control.update(Mode.FIXED_AVOID, 5000, float('inf'))

    assert abs(control.get_angle()) <= LANE_LIMIT + 1e-6


def test_the_bound_is_a_safety_net_not_a_tuning_point():
    # 킥을 고친 뒤 실측 최대는 35° 근처다. 평범한 오프셋은 상한에 닿지 않는다.
    control = controller()
    control.update(Mode.FIXED_AVOID, -289, float('inf'))

    assert abs(control.get_angle()) < LANE_LIMIT


def test_cone_drive_keeps_its_own_tighter_bound():
    control = controller()
    control.update(Mode.CONE_DRIVE, -5000, float('inf'))

    assert abs(control.get_angle()) <= CONE_LIMIT + 1e-6


def test_cone_drive_first_cycle_is_also_kick_free():
    control = controller()
    control.update(Mode.LANE_DRIVE, 0, float('inf'))

    control.update(Mode.CONE_DRIVE, 30, float('inf'))

    # kp 1.0, kd 0.0 for cones, so the kick never showed here - but the first
    # cycle must still be the proportional term alone.
    assert control.get_angle() == pytest.approx(30.0, abs=0.01)


def test_a_fresh_controller_has_no_previous_offset():
    control = controller()

    assert control.prev_offset is None
