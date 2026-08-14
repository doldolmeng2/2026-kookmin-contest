import unittest

from main.control import BEFORE, CHANGE_LANE, LANE_DRIVE, Controller


class PurePursuitTest(unittest.TestCase):
    def make_controller(self):
        return Controller(node=None)

    def test_pure_pursuit_is_zero_on_center(self):
        controller = self.make_controller()
        self.assertAlmostEqual(controller._compute_steering_pure_pursuit(0), 0.0)

    def test_pure_pursuit_is_symmetric_and_preserves_offset_sign(self):
        controller = self.make_controller()
        positive = controller._compute_steering_pure_pursuit(40)
        negative = controller._compute_steering_pure_pursuit(-40)
        self.assertGreater(positive, 0.0)
        self.assertAlmostEqual(negative, -positive)

    def test_lane_modes_use_pure_pursuit(self):
        for mode in (LANE_DRIVE, BEFORE, CHANGE_LANE):
            with self.subTest(mode=mode):
                controller = self.make_controller()
                expected = controller._compute_steering_pure_pursuit(30)
                controller.update(mode, 30, float('inf'))
                self.assertAlmostEqual(controller.get_angle(), expected)

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
