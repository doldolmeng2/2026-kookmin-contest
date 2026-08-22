"""LANE_DRIVE Pure Pursuit 조향 계약 테스트.

Pure Pursuit은 목표점이 lookahead 안쪽에 있을 때만 "이탈이 클수록 조향도 크다"가
성립한다. /lane_offset은 경로점이 아니라 횡오차이므로, 그대로 넣으면 이탈이
lookahead를 넘어가는 순간 조향이 오히려 작아진다. 그 오적용을 막는 클램프가
살아있는지 확인하는 것이 이 파일의 핵심이다.
"""

import unittest

import math
import pathlib
import re

from main.control import (
    CONTROL_PERIOD_S,
    GUARDRAIL_PARAMS,
    GUARDRAIL_PARAM_HELP,
    GUARDRAIL_TUNABLES,
    LANE_PATH_PREVIEW_PARAMS,
    PURE_PURSUIT_PARAMS,
    STEERING_FILTER_PARAMS,
    STEERING_FILTER_PARAM_HELP,
    STEERING_FILTER_TUNABLES,
    Controller,
    validate_guardrail_params,
    validate_steering_filter_params,
)
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
        # 데드밴드가 입력을 먼저 당기므로 평평해지는 지점이 그만큼 뒤로 밀린다.
        # 켜져 있든 아니든 같은 식이 성립해야 한다.
        plateau = (controller.pure_pursuit_params['lookahead_px']
                   + controller.pure_pursuit_params['offset_deadband_px'])
        at_plateau = controller._compute_steering_pure_pursuit(plateau)
        self.assertEqual(
            controller._compute_steering_pure_pursuit(plateau + 200), at_plateau)
        self.assertEqual(
            controller._compute_steering_pure_pursuit(-(plateau + 200)),
            -at_plateau)

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


class LanePathPreviewTest(unittest.TestCase):
    """먼 BEV 경로점은 선택 사항이며 LANE_DRIVE에만 선행 제어를 더한다."""

    def legacy_result(self, mode, offset, guardrail=None):
        controller = Controller()
        controller.update(mode, offset, float('inf'), 100, guardrail)
        return controller.get_angle(), controller.get_speed()

    def preview_result(self, mode, offset, preview, guardrail=None):
        controller = Controller()
        controller.update(
            mode, offset, float('inf'), 100, guardrail, preview)
        return controller.get_angle(), controller.get_speed()

    def test_none_matches_legacy_exactly(self):
        for offset in (-180, -40, 0, 40, 180):
            expected = self.legacy_result(Mode.LANE_DRIVE, offset)
            actual = self.preview_result(
                Mode.LANE_DRIVE, offset, None)
            self.assertEqual(actual, expected)

    def test_disabled_matches_legacy_exactly(self):
        baseline = Controller()
        baseline.update(Mode.LANE_DRIVE, 35, float('inf'))

        disabled = Controller()
        disabled.lane_path_preview_params['enabled'] = False
        disabled.update(
            Mode.LANE_DRIVE, 35, float('inf'), 100, None,
            (120.0, 1.0, 1.0),
        )
        self.assertEqual(disabled.get_angle(), baseline.get_angle())
        self.assertEqual(disabled.get_speed(), baseline.get_speed())

    def test_signed_preview_steers_before_current_offset_changes(self):
        right_angle, _ = self.preview_result(
            Mode.LANE_DRIVE, 0, (80.0, 0.5, 1.0))
        left_angle, _ = self.preview_result(
            Mode.LANE_DRIVE, 0, (-80.0, -0.5, 1.0))
        self.assertGreater(right_angle, 0.0)
        self.assertAlmostEqual(left_angle, -right_angle)

    def test_confidence_blends_preview_steering(self):
        base_angle, _ = self.legacy_result(Mode.LANE_DRIVE, 20)
        full_angle, _ = self.preview_result(
            Mode.LANE_DRIVE, 20, (40.0, 0.0, 1.0))
        quarter_angle, _ = self.preview_result(
            Mode.LANE_DRIVE, 20, (40.0, 0.0, 0.25))
        self.assertAlmostEqual(
            quarter_angle - base_angle,
            0.25 * (full_angle - base_angle),
        )

    def test_preview_steering_contribution_has_independent_safety_cap(self):
        base_angle, _ = self.legacy_result(Mode.LANE_DRIVE, 0)
        max_delta = LANE_PATH_PREVIEW_PARAMS['max_target_delta_px']
        capped_angle, _ = self.preview_result(
            Mode.LANE_DRIVE, 0, (max_delta, 0.0, 1.0))
        limit = capped_angle - base_angle

        for target, direction in ((100000.0, 1.0), (-100000.0, -1.0)):
            with self.subTest(target=target):
                angle, _ = self.preview_result(
                    Mode.LANE_DRIVE, 0, (target, 0.0, 1.0))
                self.assertAlmostEqual(
                    angle - base_angle,
                    direction * limit,
                )

    def test_target_delta_is_capped_relative_to_current_offset(self):
        params = LANE_PATH_PREVIEW_PARAMS
        offset = 30.0
        capped_target = offset + params['max_target_delta_px']
        at_cap = self.preview_result(
            Mode.LANE_DRIVE, offset, (capped_target, 0.0, 1.0))
        far_beyond_cap = self.preview_result(
            Mode.LANE_DRIVE, offset, (100000.0, 0.0, 1.0))
        self.assertEqual(far_beyond_cap, at_cap)

    def test_invalid_or_nonfinite_preview_is_safely_ignored(self):
        expected = self.legacy_result(Mode.LANE_DRIVE, 25)
        invalid_values = (
            (),
            (10.0, 0.0),
            ('bad', 0.0, 1.0),
            (float('nan'), 0.0, 1.0),
            (10.0, float('inf'), 1.0),
            (10.0, 0.0, float('-inf')),
            (10.0, 0.0, 0.0),
        )
        for preview in invalid_values:
            with self.subTest(preview=preview):
                self.assertEqual(
                    self.preview_result(Mode.LANE_DRIVE, 25, preview),
                    expected,
                )

    def test_preview_speed_cap_can_slow_before_large_final_angle(self):
        controller = Controller()
        params = controller.speed_params[Mode.LANE_DRIVE]
        controller.update(
            Mode.LANE_DRIVE, 0, float('inf'), 100, None,
            (controller.lane_path_preview_params['max_target_delta_px'],
             1.0, 1.0),
        )
        angle_only_speed = controller._compute_speed_from_angle(
            controller.get_angle(), params)
        self.assertLess(controller.get_speed(), angle_only_speed)
        self.assertGreaterEqual(controller.get_speed(), params.min_speed)

    def test_other_modes_ignore_preview(self):
        preview = (100000.0, 100000.0, 1.0)
        for mode in (Mode.CONE_DRIVE, Mode.FIXED_AVOID):
            with self.subTest(mode=mode):
                self.assertEqual(
                    self.preview_result(mode, 30, preview),
                    self.legacy_result(mode, 30),
                )


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
        """여유가 줄수록 반발이 세져야 한다.

        단조 증가를 전 구간에 요구하면 안 된다. gain_deg 와 max_deg 조합에
        따라 램프가 어디서 상한에 닿는지가 달라지고(예: gain 70 / max 60 /
        margin 250 이면 여유 35px 부터 포화), 그러면 튜닝만 해도 이 테스트가
        깨진다. 실제로 지켜야 하는 것은 "절대 약해지지 않는다" + "상한 아래
        구간에서는 엄밀히 커진다" 두 가지다.
        """
        controller = self.make_controller()
        # min_trust_px 아래는 좌우 판정을 못 믿어 미관측 취급이므로 제외한다.
        floor = controller.guardrail_params['min_trust_px']
        cap = abs(float(controller.guardrail_params['max_deg']))
        magnitudes = []
        for margin in (180.0, 150.0, 120.0, 90.0, 60.0, 30.0, floor):
            fresh = self.make_controller()
            magnitudes.append(abs(fresh._compute_guardrail((900.0, margin))))
        for earlier, later in zip(magnitudes, magnitudes[1:]):
            self.assertGreaterEqual(later, earlier)
        below_cap = [value for value in magnitudes if value < cap - 1e-9]
        self.assertGreater(len(below_cap), 1)
        for earlier, later in zip(below_cap, below_cap[1:]):
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


class GuardrailParameterTest(unittest.TestCase):
    """ROS 파라미터로 열어 둔 가드레일 이득.

    실차 튜닝을 재빌드 없이 하려고 연 통로다. 검증이 느슨하면 잘못된 값이
    주행 중에 그대로 들어가므로, 거절해야 할 것을 거절하는지가 핵심이다.
    """

    def test_every_param_is_tunable_and_documented(self):
        # 항목을 추가하고 spec/help 를 빠뜨리면 런치가 KeyError 로 죽는다.
        self.assertEqual(set(GUARDRAIL_TUNABLES), set(GUARDRAIL_PARAMS))
        self.assertEqual(set(GUARDRAIL_TUNABLES), set(GUARDRAIL_PARAM_HELP))

    def test_defaults_pass_their_own_validation(self):
        self.assertEqual(
            validate_guardrail_params(GUARDRAIL_PARAMS), dict(GUARDRAIL_PARAMS))

    def test_types_are_preserved(self):
        checked = validate_guardrail_params({'gain_deg': 20, 'hold_frames': 3})
        self.assertIsInstance(checked['gain_deg'], float)
        self.assertIsInstance(checked['hold_frames'], int)

    def test_unknown_name_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_guardrail_params({'gain': 20.0})

    def test_negative_gain_is_rejected(self):
        # 음수면 반발 부호가 뒤집혀 차를 실선 쪽으로 민다.
        for name in ('gain_deg', 'max_deg', 'rate_deg', 'margin_px'):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    validate_guardrail_params({name: -1.0})

    def test_zero_decay_frames_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_guardrail_params({'decay_frames': 0})

    def test_non_finite_is_rejected(self):
        for value in (float('nan'), float('inf')):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_guardrail_params({'gain_deg': value})

    def test_bool_is_rejected(self):
        # bool 은 int 의 서브클래스라 형변환만으로는 통과한다.
        with self.assertRaises(ValueError):
            validate_guardrail_params({'hold_frames': True})

    def test_apply_changes_the_ramp(self):
        controller = Controller()
        controller.guardrail_params['rate_deg'] = 1e9
        margin = controller.guardrail_params['margin_px']
        before = abs(controller._compute_guardrail((900.0, margin / 2.0)))

        weaker = Controller()
        weaker.apply_guardrail_params({'gain_deg': 1.0, 'rate_deg': 1e9})
        after = abs(weaker._compute_guardrail((900.0, margin / 2.0)))

        self.assertLess(after, before)

    def test_apply_rejects_bad_values_without_partial_writes(self):
        controller = Controller()
        original = dict(controller.guardrail_params)
        with self.assertRaises(ValueError):
            controller.apply_guardrail_params(
                {'gain_deg': 10.0, 'rate_deg': -1.0})
        self.assertEqual(controller.guardrail_params, original)

    def test_apply_keeps_accumulated_angle(self):
        # 튜닝 중 값을 바꿨다고 조향이 0으로 툭 떨어지면 그 자체가 외란이다.
        controller = Controller()
        controller._compute_guardrail((900.0, 20.0))
        pushed = controller.guardrail_angle
        self.assertNotEqual(pushed, 0.0)

        controller.apply_guardrail_params({'gain_deg': 1.0})
        self.assertEqual(controller.guardrail_angle, pushed)


class SteeringFilterTest(unittest.TestCase):
    """지터 억제(데드밴드 / 저역통과) 계약.

    이 둘의 존재 이유는 "직선 감도와 코너 권한을 분리한다" 이다. lookahead_px /
    wheelbase_px 로는 분리가 안 된다 — 비를 유지한 채 스케일만 키우면 둘이 같이
    줄어든다. 그래서 코너 조향을 거의 그대로 둔 채 작은 입력만 죽는지가 핵심이다.
    """

    CORNER_OFFSET = -180

    @staticmethod
    def all_off(**overrides):
        """모든 정형 파라미터를 끈 조합. 배포 기본값과 무관하게 만든다.

        기본값은 실차 튜닝으로 계속 바뀐다. 거기에 기대어 쓰면 튜닝할 때마다
        관계없는 테스트가 깨진다.

        offset_median_frames 만 0 이 아니라 1 이 "없음" 이다 — 창 길이라서
        0 은 의미가 깨지고 검증에서 거절된다.
        """
        params = {name: 0.0 for name in STEERING_FILTER_TUNABLES}
        params['offset_median_frames'] = 1
        params.update(overrides)
        return params

    def drive(self, offsets, **filter_params):
        controller = Controller()
        controller.apply_steering_filter_params(self.all_off(**filter_params))
        angles = []
        for offset in offsets:
            controller.update(Mode.LANE_DRIVE, offset, 999.0)
            angles.append(controller.get_angle())
        return angles

    def test_filter_params_have_one_source(self):
        """런타임 값은 pure_pursuit_params 한 곳에만 산다.

        STEERING_FILTER_PARAMS 는 ROS 파라미터 기본값을 노출하기 위한 사본일
        뿐이다. 둘이 갈라지면 launch 로 넘긴 값과 소스 기본값이 조용히 달라진다.
        """
        for name in STEERING_FILTER_TUNABLES:
            self.assertEqual(
                STEERING_FILTER_PARAMS[name], PURE_PURSUIT_PARAMS[name])
        self.assertEqual(
            dict(Controller().pure_pursuit_params, **STEERING_FILTER_PARAMS),
            dict(Controller().pure_pursuit_params),
        )

    def test_zeroed_filters_leave_steering_untouched(self):
        jitter = [4, -6, 5, -3, 7, -5, 2, -4]
        bare = Controller()
        bare.apply_steering_filter_params(self.all_off())
        angles = []
        for offset in jitter:
            bare.update(Mode.LANE_DRIVE, offset, 999.0)
            angles.append(bare.get_angle())
        self.assertEqual(self.drive(jitter), angles)

    def test_deadband_zeroes_small_offsets(self):
        angles = self.drive([4, -6, 5, -3, 7, -5], offset_deadband_px=12.0)
        self.assertEqual(angles, [0.0] * 6)

    def test_deadband_costs_far_less_at_a_corner_than_near_centre(self):
        """데드밴드의 존재 이유: 작은 입력만 골라 죽이고 코너는 남긴다.

        절대 손해율로 고정하면 lookahead/wheelbase 를 튜닝할 때마다 깨진다.
        곡선 모양과 무관하게 성립해야 하는 관계 — "중앙 근처보다 코너에서
        훨씬 덜 깎인다" — 를 대신 고정한다.
        """
        deadband = 12.0

        def cost(offset):
            plain = abs(self.drive([offset])[0])
            damped = abs(self.drive([offset], offset_deadband_px=deadband)[0])
            self.assertLessEqual(damped, plain)
            return (plain - damped) / plain

        near_centre = cost(30)
        corner = cost(self.CORNER_OFFSET)
        self.assertGreater(near_centre, 0.2)          # 중앙 근처는 확실히 깎인다
        self.assertLess(corner, near_centre / 3.0)    # 코너는 훨씬 덜 깎인다
        self.assertLess(corner, 0.15)                 # 그리고 절대값도 작다

    def test_deadband_is_continuous_at_the_boundary(self):
        """경계에서 자르지 말고 빼야 한다. clip 이면 여기서 조향이 툭 튄다."""
        controller = Controller()
        controller.apply_steering_filter_params({'offset_deadband_px': 12.0})
        just_outside = controller._compute_steering_pure_pursuit(13)
        self.assertGreater(just_outside, 0.0)
        self.assertLess(just_outside, controller._compute_steering_pure_pursuit(25))

    def test_lowpass_attenuates_alternating_noise(self):
        jitter = [7, -7] * 40
        plain = self.drive(jitter)
        damped = self.drive(jitter, offset_lpf_tau_s=0.10)
        # 정상상태 진폭으로 비교한다. 필터는 첫 값에서 출발해 수렴하므로
        # 앞쪽 과도구간을 재면 감쇠가 아니라 수렴 속도를 재게 된다.
        self.assertLess(
            max(abs(value) for value in damped[-10:]),
            max(abs(value) for value in plain[-10:]) * 0.5,
        )

    def test_lowpass_converges_to_a_held_offset(self):
        """코너처럼 값이 유지되면 결국 필터 없는 값에 수렴해야 한다."""
        held = [self.CORNER_OFFSET] * 200
        self.assertAlmostEqual(
            self.drive(held, offset_lpf_tau_s=0.10)[-1],
            self.drive(held)[-1],
            places=3,
        )

    def test_lowpass_alpha_follows_the_control_period(self):
        controller = Controller()
        controller.apply_steering_filter_params({'offset_lpf_tau_s': 0.10})
        controller._filter_lane_offset(0.0)
        controller._filter_lane_offset(100.0)
        expected = 100.0 * (1.0 - math.exp(-CONTROL_PERIOD_S / 0.10))
        self.assertAlmostEqual(controller.offset_filtered, expected, places=6)

    def test_control_period_matches_main_node_timer(self):
        """CONTROL_PERIOD_S 가 어긋나면 tau 가 초 단위라는 약속이 깨진다."""
        source = (pathlib.Path(__file__).resolve().parents[1]
                  / 'main' / 'main.py').read_text()
        found = re.search(r'create_timer\(\s*([0-9.]+)\s*,\s*self\.control_cycle',
                          source)
        self.assertIsNotNone(found)
        self.assertAlmostEqual(float(found.group(1)), CONTROL_PERIOD_S)

    def test_mode_change_clears_the_filter_state(self):
        controller = Controller()
        controller.apply_steering_filter_params({'offset_lpf_tau_s': 0.10})
        controller.update(Mode.LANE_DRIVE, -200, 999.0)
        self.assertIsNotNone(controller.offset_filtered)
        controller.update(Mode.CONE_DRIVE, 0, 999.0)
        self.assertIsNone(controller.offset_filtered)

    def test_apply_keeps_filter_state(self):
        """튜닝 중에 상태를 버리면 그 프레임에 offset 이 통째로 튄다."""
        controller = Controller()
        controller.apply_steering_filter_params({'offset_lpf_tau_s': 0.10})
        controller.update(Mode.LANE_DRIVE, -200, 999.0)
        held = controller.offset_filtered
        controller.apply_steering_filter_params({'offset_lpf_tau_s': 0.20})
        self.assertEqual(controller.offset_filtered, held)

    def test_validation_rejects_bad_values(self):
        self.assertEqual(
            validate_steering_filter_params(STEERING_FILTER_PARAMS),
            dict(STEERING_FILTER_PARAMS),
        )
        with self.assertRaises(ValueError):
            validate_steering_filter_params({'deadband': 1.0})
        for name in STEERING_FILTER_TUNABLES:
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    validate_steering_filter_params({name: -1.0})
                with self.assertRaises(ValueError):
                    validate_steering_filter_params({name: True})
        with self.assertRaises(ValueError):
            validate_steering_filter_params({'offset_lpf_tau_s': float('nan')})

    def test_speed_slew_leaves_the_servo_angle_untouched(self):
        """조향은 즉시, 속도만 부드럽게. 이 분리가 이 기능의 전부다."""
        plain = Controller()
        slewed = Controller()
        slewed.apply_steering_filter_params({'speed_angle_slew_per_s': 50.0})
        for offset in (0, -600, -600, -600):
            plain.update(Mode.LANE_DRIVE, offset, 999.0)
            slewed.update(Mode.LANE_DRIVE, offset, 999.0)
            self.assertEqual(slewed.get_angle(), plain.get_angle())
        # 조향은 같은데 속도는 아직 덜 깎여 있어야 한다.
        self.assertGreater(slewed.get_speed(), plain.get_speed())

    def test_speed_slew_disabled_by_default(self):
        plain = Controller()
        slewed = Controller()
        slewed.apply_steering_filter_params({'speed_angle_slew_per_s': 0.0})
        for offset in (0, -600, -600):
            plain.update(Mode.LANE_DRIVE, offset, 999.0)
            slewed.update(Mode.LANE_DRIVE, offset, 999.0)
        self.assertEqual(slewed.get_speed(), plain.get_speed())

    def test_speed_slew_does_not_delay_a_realistic_corner_ramp(self):
        """진짜 코너는 조향이 초당 24 정도로 쌓인다. 50/s 제한에 걸리면 안 된다.

        걸리기 시작하면 감속이 늦어져 위험해진다. 기본 권장값이 그 위에 있는지를
        고정한다.
        """
        ramp = [Mode.LANE_DRIVE] * 50
        plain, slewed = Controller(), Controller()
        slewed.apply_steering_filter_params({'speed_angle_slew_per_s': 50.0})
        for index, mode in enumerate(ramp):
            offset = -12 * index          # 50 프레임(1초)에 걸쳐 서서히 깊어짐
            plain.update(mode, offset, 999.0)
            slewed.update(mode, offset, 999.0)
        self.assertAlmostEqual(slewed.get_speed(), plain.get_speed(), places=6)

    def test_speed_slew_converges_on_a_held_angle(self):
        held = Controller()
        held.apply_steering_filter_params({'speed_angle_slew_per_s': 50.0})
        plain = Controller()
        for _ in range(400):
            held.update(Mode.LANE_DRIVE, -600, 999.0)
            plain.update(Mode.LANE_DRIVE, -600, 999.0)
        self.assertAlmostEqual(held.get_speed(), plain.get_speed(), places=6)

    def test_mode_change_clears_the_speed_slew_state(self):
        controller = Controller()
        controller.apply_steering_filter_params(
            {'speed_angle_slew_per_s': 50.0})
        controller.update(Mode.LANE_DRIVE, -600, 999.0)
        self.assertIsNotNone(controller.speed_angle)
        controller.update(Mode.CONE_DRIVE, 0, 999.0)
        self.assertIsNone(controller.speed_angle)

    def median_only(self, frames):
        controller = Controller()
        controller.apply_steering_filter_params(
            self.all_off(offset_median_frames=frames))
        return controller

    def test_median_removes_an_isolated_spike(self):
        """지터의 실제 원인 — 튀었다 돌아오는 점프 — 이 사라져야 한다."""
        controller = self.median_only(3)
        seen = [controller._median_lane_offset(value)
                for value in (100, 100, 900, 100, 100)]
        self.assertEqual(seen, [100, 100, 100, 100, 100])

    def test_median_passes_a_sustained_step(self):
        """진짜로 차선이 바뀐 것은 창 길이만큼만 늦고 그대로 통과해야 한다.

        여기서 눌러 버리면 저역통과와 다를 바가 없어진다.
        """
        controller = self.median_only(3)
        for value in (100, 100, 100):
            controller._median_lane_offset(value)
        seen = [controller._median_lane_offset(500) for _ in range(3)]
        self.assertEqual(seen[-1], 500)
        self.assertLessEqual(seen.index(500), 2)

    def test_median_disabled_by_one_frame(self):
        controller = self.median_only(1)
        for value in (100, 900, 100):
            self.assertEqual(controller._median_lane_offset(value), value)
        self.assertFalse(controller.offset_history)

    def test_median_window_shrinks_when_the_parameter_drops(self):
        """창을 줄였는데 옛 값이 남아 있으면 그만큼 오래 끌려간다."""
        controller = self.median_only(7)
        for value in range(7):
            controller._median_lane_offset(value)
        controller.apply_steering_filter_params({'offset_median_frames': 3})
        controller._median_lane_offset(100)
        self.assertLessEqual(len(controller.offset_history), 3)

    def adaptive(self, **overrides):
        controller = Controller()
        controller.apply_steering_filter_params(self.all_off(**overrides))
        return controller

    def test_adaptive_lookahead_shortens_with_offset(self):
        controller = self.adaptive(
            lookahead_corner_px=315.0,
            adaptive_offset_start_px=100.0,
            adaptive_offset_end_px=250.0,
        )
        base = controller.pure_pursuit_params['lookahead_px']
        self.assertEqual(controller._adaptive_lookahead(0), base)
        self.assertEqual(controller._adaptive_lookahead(100), base)
        self.assertEqual(controller._adaptive_lookahead(250), 315.0)
        self.assertEqual(controller._adaptive_lookahead(9999), 315.0)
        middle = controller._adaptive_lookahead(175)
        self.assertLess(middle, base)
        self.assertGreater(middle, 315.0)

    def test_adaptive_lookahead_is_continuous_at_both_ends(self):
        """계단 전환은 문턱에서 조향을 도약시킨다. 그래서 선형으로 섞는다."""
        controller = self.adaptive(
            lookahead_corner_px=315.0,
            adaptive_offset_start_px=100.0,
            adaptive_offset_end_px=250.0,
        )
        for edge in (100.0, 250.0):
            below = controller._compute_steering_pure_pursuit(edge - 1)
            above = controller._compute_steering_pure_pursuit(edge + 1)
            self.assertLess(abs(above - below), 1.0)

    def test_adaptive_lookahead_raises_corner_steering(self):
        fixed = self.adaptive(lookahead_corner_px=0.0)
        adaptive = self.adaptive(
            lookahead_corner_px=315.0,
            adaptive_offset_start_px=100.0,
            adaptive_offset_end_px=250.0,
        )
        # 직선 근처는 그대로여야 한다 — 지터를 늘리지 않는 것이 전제다.
        self.assertAlmostEqual(
            adaptive._compute_steering_pure_pursuit(50),
            fixed._compute_steering_pure_pursuit(50), places=6)
        # 코너는 확실히 더 꺾어야 한다.
        self.assertGreater(
            abs(adaptive._compute_steering_pure_pursuit(300)),
            abs(fixed._compute_steering_pure_pursuit(300)) * 1.5)

    def test_adaptive_disabled_by_zero_corner(self):
        controller = self.adaptive(lookahead_corner_px=0.0)
        base = controller.pure_pursuit_params['lookahead_px']
        for offset in (0, 200, 5000):
            self.assertEqual(controller._adaptive_lookahead(offset), base)

    def test_adaptive_ignores_an_inverted_ramp(self):
        controller = self.adaptive(
            lookahead_corner_px=315.0,
            adaptive_offset_start_px=250.0,
            adaptive_offset_end_px=100.0,
        )
        base = controller.pure_pursuit_params['lookahead_px']
        self.assertEqual(controller._adaptive_lookahead(200), base)

    def test_steering_still_grows_with_offset_under_adaptive(self):
        """lookahead 가 같이 움직여도 단조성은 깨지면 안 된다."""
        controller = self.adaptive(
            lookahead_corner_px=315.0,
            adaptive_offset_start_px=100.0,
            adaptive_offset_end_px=250.0,
        )
        angles = [controller._compute_steering_pure_pursuit(offset)
                  for offset in range(0, 400, 5)]
        for earlier, later in zip(angles, angles[1:]):
            self.assertGreaterEqual(later, earlier)

    def test_every_tunable_is_documented(self):
        self.assertEqual(
            set(STEERING_FILTER_PARAM_HELP), set(STEERING_FILTER_TUNABLES))


if __name__ == '__main__':
    unittest.main()
