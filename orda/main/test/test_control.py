"""LANE_DRIVE Pure Pursuit 조향 계약 테스트.

Pure Pursuit은 목표점이 lookahead 안쪽에 있을 때만 "이탈이 클수록 조향도 크다"가
성립한다. /lane_offset은 경로점이 아니라 횡오차이므로, 그대로 넣으면 이탈이
lookahead를 넘어가는 순간 조향이 오히려 작아진다. 그 오적용을 막는 클램프가
살아있는지 확인하는 것이 이 파일의 핵심이다.
"""

import unittest

from main.control import Controller
from main.race_fsm import Mode


class PurePursuitTest(unittest.TestCase):
    def make_controller(self):
        return Controller()

    def test_pure_pursuit_is_zero_on_center(self):
        controller = self.make_controller()
        self.assertAlmostEqual(controller._compute_steering_pure_pursuit(0), 0.0)

    def test_pure_pursuit_is_symmetric_and_preserves_offset_sign(self):
        controller = self.make_controller()
        positive = controller._compute_steering_pure_pursuit(40)
        negative = controller._compute_steering_pure_pursuit(-40)
        self.assertGreater(positive, 0.0)
        self.assertAlmostEqual(negative, -positive)

    def test_lane_drive_uses_pure_pursuit(self):
        controller = self.make_controller()
        expected = controller._compute_steering_pure_pursuit(30)
        controller.update(Mode.LANE_DRIVE, 30, float('inf'))
        self.assertAlmostEqual(controller.get_angle(), expected)

    # ── [PD 롤백 테스트용 보관] LANE_DRIVE가 PD를 쓸 때의 계약 ─────────────
    # LANE_DRIVE를 PD로 되돌릴 때 아래 테스트를 살리고 위 Pure Pursuit 테스트를
    # 주석 처리한다. control.py의 PD_PARAMS[Mode.LANE_DRIVE]도 함께 살려야 한다.
    # def test_lane_drive_uses_pd(self):
    #     controller = self.make_controller()
    #     expected = controller._compute_steering_pd(Mode.LANE_DRIVE, 30)
    #
    #     fresh = self.make_controller()
    #     fresh.update(Mode.LANE_DRIVE, 30, float('inf'))
    #     self.assertAlmostEqual(fresh.get_angle(), expected)
    # ──────────────────────────────────────────────────────────────────────

    def test_pure_pursuit_respects_steering_limit(self):
        # 리터럴 40.0 을 기대하던 테스트였는데, max_steering_angle 이 100 으로
        # 바뀌었을 때 같이 고쳐지지 않아 계속 실패하고 있었다. 이제 파라미터에서
        # 한계를 읽어 재튜닝에도 깨지지 않게 한다.
        controller = self.make_controller()
        limit = controller.pure_pursuit_params['max_steering_angle']
        controller.pure_pursuit_params['steering_gain'] = 100.0
        self.assertEqual(controller._compute_steering_pure_pursuit(80), limit)
        self.assertEqual(controller._compute_steering_pure_pursuit(-80), -limit)

    def test_large_offset_does_not_reduce_steering(self):
        controller = self.make_controller()
        lookahead = controller.pure_pursuit_params['lookahead_px']
        at_lookahead = controller._compute_steering_pure_pursuit(lookahead)
        self.assertEqual(controller._compute_steering_pure_pursuit(800), at_lookahead)
        self.assertEqual(controller._compute_steering_pure_pursuit(-800), -at_lookahead)

    def test_steering_grows_monotonically_with_offset(self):
        controller = self.make_controller()
        angles = [
            controller._compute_steering_pure_pursuit(offset)
            for offset in (0, 20, 40, 60, 80, 100, 120)
        ]
        for earlier, later in zip(angles, angles[1:]):
            self.assertGreater(later, earlier)


class FixedAvoidStillUsesPdTest(unittest.TestCase):
    """FIXED_AVOID는 회피 전용 PD 이득을 그대로 쓴다 (Pure Pursuit 아님)."""

    def test_fixed_avoid_matches_pd_profile(self):
        controller = Controller()
        expected = controller._compute_steering_pd(Mode.FIXED_AVOID, 30)

        fresh = Controller()
        fresh.update(Mode.FIXED_AVOID, 30, float('inf'))
        self.assertAlmostEqual(fresh.get_angle(), expected)


class GuardrailTest(unittest.TestCase):
    """바깥 실선 반발항.

    Pure Pursuit은 offset이 아무리 커도 38.9를 넘지 못한다(atan2가 x=lookahead에서
    꼭짓점을 찍고, lateral clamp가 정확히 거기 걸려 있다). 코너에서 조향이 모자라
    차선을 벗어나는 문제의 실제 원인이 이것이므로, 이 항은 그 상한을 넘길 수 있어야
    한다. 동시에 여유가 충분할 때는 정확히 0이어서 직선 주행을 건드리지 않아야 한다.
    """

    def make_controller(self):
        controller = Controller()
        # 변화율 제한 때문에 한 프레임으로는 목표값에 못 닿는다. 램프 자체를
        # 검증하는 테스트에서는 제한을 풀어 둔다(제한은 별도 테스트로 확인).
        controller.guardrail_params['rate_deg'] = 1e9
        return controller

    def far_from_both_rails(self):
        threshold = Controller().guardrail_params['margin_px']
        return (threshold + 50.0, threshold + 50.0)

    # ── 기준선: 꺼져 있으면 기존 거동과 완전히 같아야 한다 ─────────────────
    def test_disabled_guardrail_matches_pure_pursuit_exactly(self):
        for offset in (-260, -120, -40, 0, 40, 120, 260, 800):
            baseline = Controller()
            expected = baseline._compute_steering_pure_pursuit(offset)

            fresh = Controller()
            fresh.update(Mode.LANE_DRIVE, offset, float('inf'), 100, None)
            self.assertEqual(fresh.get_angle(), expected)

    def test_zero_margin_px_disables_the_term(self):
        controller = self.make_controller()
        controller.guardrail_params['margin_px'] = 0.0
        expected = controller._compute_steering_pure_pursuit(30)

        controller.update(Mode.LANE_DRIVE, 30, float('inf'), 100, (5.0, 5.0))
        self.assertEqual(controller.get_angle(), expected)

    def test_ample_margin_produces_exactly_zero(self):
        controller = self.make_controller()
        expected = controller._compute_steering_pure_pursuit(30)

        controller.update(
            Mode.LANE_DRIVE, 30, float('inf'), 100, self.far_from_both_rails())
        self.assertEqual(controller.get_angle(), expected)

    # ── 방향 ──────────────────────────────────────────────────────────────
    def test_near_right_rail_steers_left(self):
        controller = self.make_controller()
        margin = controller.guardrail_params['margin_px']
        # 오른쪽만 가깝다 → 좌조향(음의 기여)
        self.assertLess(controller._compute_guardrail((margin + 200.0, 20.0)), 0.0)

    def test_near_left_rail_steers_right(self):
        controller = self.make_controller()
        margin = controller.guardrail_params['margin_px']
        self.assertGreater(controller._compute_guardrail((20.0, margin + 200.0)), 0.0)

    def test_nearest_rail_wins_when_both_are_close(self):
        controller = self.make_controller()
        # 오른쪽이 더 가까우므로 왼쪽 실선이 보여도 좌조향이어야 한다.
        self.assertLess(controller._compute_guardrail((100.0, 40.0)), 0.0)

        other = self.make_controller()
        self.assertGreater(other._compute_guardrail((40.0, 100.0)), 0.0)

    # ── 크기 ──────────────────────────────────────────────────────────────
    def test_term_grows_as_margin_shrinks(self):
        controller = self.make_controller()
        magnitudes = []
        # min_trust_px 아래는 좌우 판정을 못 믿어 미관측 취급이므로 제외한다.
        floor = controller.guardrail_params['min_trust_px']
        for margin in (180.0, 150.0, 120.0, 90.0, 60.0, 30.0, floor):
            fresh = self.make_controller()
            magnitudes.append(abs(fresh._compute_guardrail((900.0, margin))))
        for earlier, later in zip(magnitudes, magnitudes[1:]):
            self.assertGreater(later, earlier)

    def test_term_is_capped_by_max_deg(self):
        controller = self.make_controller()
        controller.guardrail_params['gain_deg'] = 500.0
        controller.guardrail_params['max_deg'] = 20.0
        self.assertAlmostEqual(
            controller._compute_guardrail((900.0, 20.0)), -20.0)

    def test_guardrail_can_exceed_pure_pursuit_ceiling(self):
        """이 항의 존재 이유. 합계가 Pure Pursuit 상한을 넘어야 한다."""
        ceiling = max(
            abs(Controller()._compute_steering_pure_pursuit(offset))
            for offset in range(0, 1200, 10)
        )
        # 오른쪽 실선에 붙었고 중앙선 오차도 같은 방향을 가리키는 상황.
        # 예전에는 -38.9 가 한계였지만 이제 그 아래로 더 꺾을 수 있어야 한다.
        controller = self.make_controller()
        controller.update(Mode.LANE_DRIVE, -260, float('inf'), 100, (900.0, 20.0))
        self.assertLess(controller.get_angle(), -ceiling)

    def test_total_angle_respects_pure_pursuit_limit(self):
        controller = self.make_controller()
        controller.guardrail_params['gain_deg'] = 5000.0
        controller.guardrail_params['max_deg'] = 5000.0
        limit = controller.pure_pursuit_params['max_steering_angle']

        controller.update(Mode.LANE_DRIVE, 260, float('inf'), 100, (20.0, 900.0))
        self.assertLessEqual(abs(controller.get_angle()), limit)

    # ── 미관측/변화율/감쇠 ────────────────────────────────────────────────
    def test_negative_margins_mean_unobserved(self):
        controller = self.make_controller()
        self.assertEqual(controller._compute_guardrail((-1.0, -1.0)), 0.0)

    def test_margin_below_trust_floor_is_treated_as_unobserved(self):
        """실선 위에 올라타면 같은 선이 반대쪽에서 보여 부호가 뒤집힌다.

        carlane1 bag t=38.4~38.6s 에서 여유가 (미관측, 1px) → (1px, 미관측) 으로
        뒤집혔고, 그대로 쓰면 트랙 밖으로 나간 차를 더 밖으로 밀었다.
        """
        controller = self.make_controller()
        floor = controller.guardrail_params['min_trust_px']

        self.assertIsNone(controller._guardrail_target(
            (-1.0, floor - 1.0), controller.guardrail_params['margin_px'],
            controller.guardrail_params))
        self.assertIsNotNone(controller._guardrail_target(
            (-1.0, floor + 1.0), controller.guardrail_params['margin_px'],
            controller.guardrail_params))

    def test_sign_flip_on_the_line_does_not_reverse_steering(self):
        controller = self.make_controller()
        # 오른쪽 실선에 접근하다가 그 위에 올라타는 순간을 흉내낸다.
        controller._compute_guardrail((-1.0, 40.0))
        pushed = controller.guardrail_angle
        self.assertLess(pushed, 0.0)

        # 여유가 1px 로 뒤집혀도 부호가 반대로 튀면 안 된다.
        self.assertEqual(controller._compute_guardrail((1.0, -1.0)), pushed)
        self.assertEqual(controller._compute_guardrail((-1.0, 1.0)), pushed)

    def test_one_unobserved_side_still_uses_the_other(self):
        controller = self.make_controller()
        self.assertLess(controller._compute_guardrail((-1.0, 30.0)), 0.0)

    def test_rate_limit_caps_per_frame_change(self):
        controller = Controller()          # 변화율 제한을 그대로 둔다
        rate = controller.guardrail_params['rate_deg']
        first = controller._compute_guardrail((900.0, 20.0))
        self.assertAlmostEqual(abs(first), rate)
        second = controller._compute_guardrail((900.0, 20.0))
        self.assertAlmostEqual(abs(second), 2 * rate)

    def test_lost_rail_holds_then_decays_to_zero(self):
        controller = self.make_controller()
        held = controller._compute_guardrail((900.0, 20.0))
        self.assertNotEqual(held, 0.0)

        hold = int(controller.guardrail_params['hold_frames'])
        for _ in range(hold):
            self.assertEqual(controller._compute_guardrail(None), held)

        decay = int(controller.guardrail_params['decay_frames'])
        for _ in range(decay):
            controller._compute_guardrail(None)
        self.assertEqual(controller._compute_guardrail(None), 0.0)

    def test_reset_clears_guardrail_state(self):
        controller = self.make_controller()
        controller._compute_guardrail((900.0, 20.0))
        controller.reset()
        self.assertEqual(controller.guardrail_angle, 0.0)
        self.assertEqual(controller.guardrail_missing_frames, 0)

    def test_mode_switch_resets_guardrail(self):
        controller = self.make_controller()
        controller.update(Mode.LANE_DRIVE, 0, float('inf'), 100, (900.0, 20.0))
        self.assertNotEqual(controller.guardrail_angle, 0.0)

        controller.update(Mode.FIXED_AVOID, 0, float('inf'))
        self.assertEqual(controller.guardrail_angle, 0.0)

    # ── 속도 결합 ─────────────────────────────────────────────────────────
    def test_speed_reflects_the_guardrail_augmented_angle(self):
        """가드레일로 더 꺾었으면 그만큼 더 감속해야 한다."""
        plain = Controller()
        plain.update(Mode.LANE_DRIVE, 0, float('inf'), 100, None)

        pushed = self.make_controller()
        pushed.update(Mode.LANE_DRIVE, 0, float('inf'), 100, (900.0, 20.0))

        self.assertGreater(abs(pushed.get_angle()), abs(plain.get_angle()))
        self.assertLess(pushed.get_speed(), plain.get_speed())

    # ── 다른 모드는 영향 없음 ─────────────────────────────────────────────
    def test_cone_drive_ignores_guardrail(self):
        controller = Controller()
        expected = controller._compute_steering_pd(Mode.CONE_DRIVE, 30)

        fresh = Controller()
        fresh.update(Mode.CONE_DRIVE, 30, float('inf'), 100, (0.0, 0.0))
        self.assertAlmostEqual(fresh.get_angle(), expected)

    def test_fixed_avoid_ignores_guardrail(self):
        controller = Controller()
        expected = controller._compute_steering_pd(Mode.FIXED_AVOID, 30)

        fresh = Controller()
        fresh.update(Mode.FIXED_AVOID, 30, float('inf'), 100, (0.0, 0.0))
        self.assertAlmostEqual(fresh.get_angle(), expected)


if __name__ == '__main__':
    unittest.main()
