"""Unit tests for the pure interfering-vehicle avoidance/overtake logic."""

import pytest

from main.overtake import OvertakeConfig, OvertakeGuard


INF = float("inf")


def guard(**kwargs):
    return OvertakeGuard(OvertakeConfig(**kwargs)) if kwargs else OvertakeGuard()


# ── 설정 검증 ────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "field", ["trigger_box_px", "side_detect_m", "pass_delay_s", "zone_timeout_s"]
)
def test_config_rejects_negative(field):
    with pytest.raises(ValueError):
        OvertakeConfig(**{field: -1.0})


# ── 회피 방향 ────────────────────────────────────────────────────────────────


def test_avoid_direction_is_opposite_of_obstacle_lane():
    g = guard()
    # 장애물이 Lane2(오른쪽, car_lane=2) → Lane1(0)으로 피한다
    assert g.avoid_target_lane(car_lane=2, lane_target=1) == 0
    # 장애물이 Lane1(왼쪽, car_lane=1) → Lane2(1)로 피한다
    assert g.avoid_target_lane(car_lane=1, lane_target=0) == 1


def test_avoid_direction_falls_back_to_opposite_when_lane_unknown():
    g = guard()
    for unknown in (-1, 0):
        assert g.avoid_target_lane(car_lane=unknown, lane_target=1) == 0
        assert g.avoid_target_lane(car_lane=unknown, lane_target=0) == 1


def test_does_not_toggle_back_into_the_lane_it_just_avoided():
    """충돌의 근본 원인이었던 맹목적 토글에 대한 회귀 테스트.

    Lane1(0)으로 피한 뒤에도 오른쪽 차선(car_lane=2)의 앞차가 계속 보이는데,
    여기서 다시 토글하면 방금 피한 차선으로 되돌아가 충돌한다.
    """
    g = guard()
    assert g.begin_avoidance(
        box_size=2100, car_lane=2, lane_target=1, detected_lane=1
    ) == 0
    # 이미 회피 차선에 있으므로 더 이상 차선을 바꾸지 않는다
    assert g.begin_avoidance(
        box_size=2100, car_lane=2, lane_target=0, detected_lane=1
    ) is None


def test_no_avoidance_below_box_threshold_or_in_other_lane():
    g = guard()
    assert g.begin_avoidance(
        box_size=1899, car_lane=2, lane_target=1, detected_lane=1
    ) is None
    # 장애물이 반대 차선(car_lane=1 → Lane1)이고 우리는 Lane2(1)
    assert g.begin_avoidance(
        box_size=5000, car_lane=1, lane_target=1, detected_lane=1
    ) is None


def test_unknown_car_lane_is_treated_as_ego_lane():
    """확정할 수 없으면 회피하는 쪽이 안전하다 (README 정책)."""
    g = guard()
    for unknown in (-1, 0):
        assert g.obstacle_in_ego_lane(unknown, lane_target=1, detected_lane=1) is True


def test_detected_lane_wins_over_lane_target():
    g = guard()
    # 실측이 Lane1(0)인데 목표는 Lane2(1). 장애물은 Lane1(car_lane=1).
    assert g.obstacle_in_ego_lane(1, lane_target=1, detected_lane=0) is True
    assert g.obstacle_in_ego_lane(2, lane_target=1, detected_lane=0) is False


# ── 추월 완료 및 복귀 ────────────────────────────────────────────────────────


def test_overtake_completes_only_after_pass_delay():
    g = guard()
    g.begin_avoidance(box_size=2100, car_lane=2, lane_target=1, detected_lane=1)
    g.enter_zone(now=0.0)

    # 옆이 비어 있는 동안은 완료가 아니다
    assert g.update_zone(
        now=1.0, lane_target=0, side_left=INF, side_right=INF
    ).complete is False

    # Lane1(0) 주행 중이므로 오른쪽에서 인식한다
    seen = g.update_zone(now=2.0, lane_target=0, side_left=INF, side_right=0.42)
    assert seen.side_just_seen is True
    assert seen.complete is False
    assert seen.side_distance == pytest.approx(0.42)

    # 2초 지나기 전에는 아직 완료가 아니다
    assert g.update_zone(
        now=3.9, lane_target=0, side_left=INF, side_right=INF
    ).complete is False
    # 2초가 지나면 완료
    assert g.update_zone(
        now=4.0, lane_target=0, side_left=INF, side_right=INF
    ).complete is True


def test_side_detection_watches_the_lane_we_came_from():
    g = guard()
    # Lane2(1) 주행 중이면 왼쪽을 본다. 오른쪽 값은 무시되어야 한다.
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=0.1, lane_target=1, side_left=INF, side_right=0.2
    ).side_just_seen is False
    assert g.update_zone(
        now=0.2, lane_target=1, side_left=0.3, side_right=INF
    ).side_just_seen is True


def test_side_just_seen_fires_once():
    g = guard()
    g.enter_zone(now=0.0)
    first = g.update_zone(now=0.1, lane_target=0, side_left=INF, side_right=0.4)
    second = g.update_zone(now=0.2, lane_target=0, side_left=INF, side_right=0.4)
    assert first.side_just_seen is True
    assert second.side_just_seen is False


def test_restores_the_lane_it_started_from():
    g = guard()
    target = g.begin_avoidance(
        box_size=2100, car_lane=2, lane_target=1, detected_lane=1
    )
    assert target == 0
    assert g.take_restore_lane() == 1
    # 한 번 소비하면 사라진다 (같은 회피로 두 번 복귀하지 않는다)
    assert g.take_restore_lane() is None


def test_enter_zone_keeps_the_restore_lane():
    """회피 → 구간 진입 순서라, 구간 진입이 복귀 차선을 지우면 안 된다."""
    g = guard()
    g.begin_avoidance(box_size=2100, car_lane=2, lane_target=1, detected_lane=1)
    g.enter_zone(now=0.0)
    assert g.take_restore_lane() == 1


def test_zone_timeout_is_the_escape_hatch():
    """방해차량을 못 만난 바퀴에서도 구간을 빠져나가야 한다 (README)."""
    g = guard(zone_timeout_s=12.0)
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=11.9, lane_target=0, side_left=INF, side_right=INF
    ).complete is False
    late = g.update_zone(now=12.0, lane_target=0, side_left=INF, side_right=INF)
    assert late.complete is True
    assert late.timed_out is True


def test_stuck_side_sensor_cannot_complete_before_the_delay():
    """고착된 센서가 있어도 지연 시간 자체는 우회할 수 없다.

    초음파가 4cm에 고착되어 차선 변경 직후 곧바로 '추월 완료'가 나던
    문제에 대한 회귀 테스트.
    """
    g = guard(pass_delay_s=2.0)
    g.begin_avoidance(box_size=2100, car_lane=2, lane_target=1, detected_lane=1)
    g.enter_zone(now=0.0)
    # 진입 직후부터 계속 0.04m로 붙어 있어도 2초 전에는 완료되지 않는다
    for t in (0.02, 0.5, 1.0, 1.99):
        assert g.update_zone(
            now=t, lane_target=0, side_left=INF, side_right=0.04
        ).complete is False
    # 최초 인식이 t=0.02이므로 완료 시점은 2.02이다
    assert g.update_zone(
        now=2.03, lane_target=0, side_left=INF, side_right=0.04
    ).complete is True


def test_reset_clears_everything():
    g = guard()
    g.begin_avoidance(box_size=2100, car_lane=2, lane_target=1, detected_lane=1)
    g.enter_zone(now=0.0)
    g.update_zone(now=0.1, lane_target=0, side_left=INF, side_right=0.4)
    g.reset()
    assert g.lane_before_avoid is None
    assert g.side_seen_at is None
    assert g.zone_entered_at is None
