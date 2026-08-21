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
        controller = self.make_controller()
        controller.pure_pursuit_params['steering_gain'] = 100.0
        self.assertEqual(controller._compute_steering_pure_pursuit(80), 40.0)
        self.assertEqual(controller._compute_steering_pure_pursuit(-80), -40.0)

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


if __name__ == '__main__':
    unittest.main()
