"""Unit tests for the pure overtake-completion logic."""

import pytest

from main.overtake import OvertakeConfig, OvertakeGuard


INF = float("inf")

# "옆에 방해차량이 있다"를 표현하는 거리(m). 고정장애물 bag 6개에서 통과 순간
# 감시 측면 최소거리가 0.26~0.34 m 로 측정됐고, 그 안쪽 값을 쓴다. 예전에는
# 0.42/0.40 을 썼는데 side_detect_m 을 0.40 으로 조이자 임계값 위로 올라가
# "차를 봤다"를 표현하지 못했다. 임계값을 만질 때 같이 검토할 것.
CAR_SIDE_M = 0.30


def guard(**kwargs):
    return OvertakeGuard(OvertakeConfig(**kwargs)) if kwargs else OvertakeGuard()


@pytest.mark.parametrize("field", ["side_detect_m", "pass_delay_s", "zone_timeout_s"])
def test_config_rejects_negative(field):
    with pytest.raises(ValueError):
        OvertakeConfig(**{field: -1.0})


def test_completes_only_after_pass_delay():
    g = guard()
    g.enter_zone(now=0.0)

    # 옆이 비어 있는 동안은 완료가 아니다
    assert g.update_zone(
        now=1.0, lane_target=1, side_left=INF, side_right=INF
    ).complete is False

    # 1차선(1) 주행 중이므로 오른쪽에서 인식한다
    seen = g.update_zone(
        now=2.0, lane_target=1, side_left=INF, side_right=CAR_SIDE_M
    )
    assert seen.side_just_seen is True
    assert seen.complete is False
    assert seen.side_distance == pytest.approx(CAR_SIDE_M)

    cleared = g.update_zone(
        now=3.0, lane_target=1, side_left=INF, side_right=INF
    )
    assert cleared.side_just_cleared is True
    assert cleared.complete is False
    assert g.update_zone(
        now=4.9, lane_target=1, side_left=INF, side_right=INF
    ).complete is False
    assert g.update_zone(
        now=5.0, lane_target=1, side_left=INF, side_right=INF
    ).complete is True


def test_watches_the_side_we_came_from():
    """2차선(2) 주행 중이면 왼쪽을 본다. 오른쪽 값은 무시되어야 한다."""
    g = guard()
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=0.1, lane_target=2, side_left=INF, side_right=0.2
    ).side_just_seen is False
    assert g.update_zone(
        now=0.2, lane_target=2, side_left=0.3, side_right=INF
    ).side_just_seen is True


def test_side_just_seen_fires_once():
    g = guard()
    g.enter_zone(now=0.0)
    first = g.update_zone(
        now=0.1, lane_target=1, side_left=INF, side_right=CAR_SIDE_M
    )
    second = g.update_zone(
        now=0.2, lane_target=1, side_left=INF, side_right=CAR_SIDE_M
    )
    assert first.side_just_seen is True
    assert second.side_just_seen is False


def test_zone_timeout_reports_but_does_not_blindly_complete():
    """측면 통과 증거가 없으면 시간 초과도 주행 상태를 바꾸지 않는다."""
    g = guard(zone_timeout_s=12.0)
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=11.9, lane_target=1, side_left=INF, side_right=INF
    ).complete is False
    late = g.update_zone(now=12.0, lane_target=1, side_left=INF, side_right=INF)
    assert late.complete is False
    assert late.timed_out is True
    # 경고는 한 번만 보고한다.
    repeated = g.update_zone(
        now=12.1, lane_target=1, side_left=INF, side_right=INF
    )
    assert repeated.complete is False
    assert repeated.timed_out is False


def test_stuck_side_sensor_never_completes_without_clearance():
    """고착된 센서는 사라짐 에지가 없으므로 완료할 수 없다.

    초음파가 4cm에 고착되어 차선 변경 직후 곧바로 '추월 완료'가 나던 문제에
    대한 회귀 테스트.
    """
    g = guard(pass_delay_s=2.0)
    g.enter_zone(now=0.0)
    for t in (0.02, 0.5, 1.0, 1.99):
        assert g.update_zone(
            now=t, lane_target=1, side_left=INF, side_right=0.04
        ).complete is False
    assert g.update_zone(
        now=2.03, lane_target=1, side_left=INF, side_right=0.04
    ).complete is False


def test_reappearance_resets_continuous_clear_timer():
    g = guard(pass_delay_s=2.0)
    g.enter_zone(now=0.0)
    g.update_zone(now=0.1, lane_target=1, side_left=INF, side_right=CAR_SIDE_M)
    g.update_zone(now=1.0, lane_target=1, side_left=INF, side_right=INF)
    g.update_zone(now=2.0, lane_target=1, side_left=INF, side_right=CAR_SIDE_M)
    g.update_zone(now=3.0, lane_target=1, side_left=INF, side_right=INF)
    assert g.update_zone(
        now=4.9, lane_target=1, side_left=INF, side_right=INF
    ).complete is False
    assert g.update_zone(
        now=5.0, lane_target=1, side_left=INF, side_right=INF
    ).complete is True


def test_measured_lane_wins_over_lane_target_for_side_choice():
    """실측 차선(/lane_position)이 있으면 그쪽을 보고, 없으면 목표로 폴백한다."""
    # 명령은 2차선(2)이지만 실측은 아직 1차선(1) → 오른쪽을 봐야 한다
    g = guard()
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=0.1, lane_target=2, side_left=INF, side_right=CAR_SIDE_M, ego_lane=1
    ).side_just_seen is True

    # 실측이 미확정이면 목표(2차선)를 따라 왼쪽을 본다
    g2 = guard()
    g2.enter_zone(now=0.0)
    assert g2.update_zone(
        now=0.1, lane_target=2, side_left=INF, side_right=CAR_SIDE_M, ego_lane=-1
    ).side_just_seen is False
    assert g2.update_zone(
        now=0.2, lane_target=2, side_left=CAR_SIDE_M, side_right=INF, ego_lane=-1
    ).side_just_seen is True


def test_reset_and_enter_zone_clear_state():
    g = guard()
    g.enter_zone(now=0.0)
    g.update_zone(now=0.1, lane_target=1, side_left=INF, side_right=CAR_SIDE_M)
    assert g.side_seen_at is not None

    g.enter_zone(now=5.0)
    assert g.side_seen_at is None
    assert g.zone_elapsed(6.0) == pytest.approx(1.0)

    g.reset()
    assert g.zone_entered_at is None
    assert g.zone_elapsed(6.0) is None


def test_invalidate_clearance_preserves_seen_and_zone_state():
    g = guard()
    g.enter_zone(now=0.0)
    g.update_zone(now=0.1, lane_target=1, side_left=INF, side_right=CAR_SIDE_M)
    g.update_zone(now=0.2, lane_target=1, side_left=INF, side_right=INF)

    g.invalidate_clearance()

    assert g.clear_started_at is None
    assert g.side_seen_at == pytest.approx(0.1)
    assert g.side_seen_distance == pytest.approx(CAR_SIDE_M)
    assert g.zone_entered_at == pytest.approx(0.0)


def test_center_lane_has_no_side_to_watch():
    """중앙(0) 주행 중에는 감시할 반대편이 정해지지 않는다."""
    g = guard()
    g.enter_zone(now=0.0)
    assert g.update_zone(
        now=0.1, lane_target=0, side_left=0.1, side_right=0.1
    ).side_just_seen is False
