import unittest

import numpy as np

from segmentation_tools.core import (ROAD_CLASS, SHORTCUT_CLASS,
                                     filter_center_lane_off_surface)


def frame_with(surface_class=None):
    """배경뿐인 360x640 클래스 맵. surface_class 를 주면 가운데를 그 주행면으로 채운다."""
    label = np.zeros((360, 640), dtype=np.uint8)
    if surface_class is not None:
        label[200:340, 180:460] = surface_class
    return label


def road_and_shortcut_frame():
    """왼쪽에 road, 오른쪽에 shortcut 이 서로 떨어져 있는 클래스 맵."""
    label = np.zeros((360, 640), dtype=np.uint8)
    label[200:340, 40:280] = ROAD_CLASS
    label[200:340, 360:600] = SHORTCUT_CLASS
    return label


class FilterCenterLaneOffSurfaceTest(unittest.TestCase):
    def test_keeps_center_lane_drawn_on_road(self):
        label = frame_with(4)
        label[220:320, 310:330] = 1
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 0)
        self.assertTrue(np.array_equal(filtered, label))

    def test_keeps_center_lane_drawn_on_shortcut(self):
        label = frame_with(5)
        label[220:320, 310:330] = 1
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 0)
        self.assertEqual(np.count_nonzero(filtered == 1), 100 * 20)

    def test_removes_center_lane_floating_outside_the_track(self):
        label = frame_with(4)
        label[40:90, 20:40] = 1          # 트랙 밖, 주변이 전부 background
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 50 * 20)
        self.assertEqual(np.count_nonzero(filtered == 1), 0)
        self.assertEqual(np.count_nonzero(filtered == 4), np.count_nonzero(label == 4))

    def test_keeps_real_lane_and_drops_the_phantom_in_one_frame(self):
        label = frame_with(4)
        label[220:320, 310:330] = 1      # 도로 위 진짜 중앙선
        label[40:90, 20:40] = 1          # 트랙 밖 유령 중앙선
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 50 * 20)
        self.assertEqual(np.count_nonzero(filtered == 1), 100 * 20)
        self.assertTrue(np.all(filtered[220:320, 310:330] == 1))
        self.assertTrue(np.all(filtered[40:90, 20:40] == 0))

    def test_removes_every_phantom_without_touching_the_caller_array(self):
        """유령이 둘 이상이어도 모두 지우고, 입력 배열은 그대로 남겨야 한다."""
        label = frame_with(4)
        label[220:320, 310:330] = 1      # 도로 위 진짜 중앙선
        label[40:90, 20:40] = 1          # 유령 1
        label[40:90, 580:600] = 1        # 유령 2
        original = label.copy()
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 2 * 50 * 20)
        self.assertEqual(np.count_nonzero(filtered == 1), 100 * 20)
        self.assertTrue(np.array_equal(label, original))

    def test_center_lane_half_off_the_road_edge_is_dropped(self):
        """도로 경계 밖으로 벗어난 성분은 테두리 절반 이상이 background 라 지워진다."""
        label = frame_with(4)
        label[220:320, 120:140] = 1      # 도로(x>=180) 왼쪽 바깥
        _, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 100 * 20)

    def test_disabled_by_zero_radius_returns_input_untouched(self):
        label = frame_with(4)
        label[40:90, 20:40] = 1
        filtered, removed = filter_center_lane_off_surface(label, radius=0)
        self.assertIs(filtered, label)
        self.assertEqual(removed, 0)

    def test_disabled_by_zero_ratio_returns_input_untouched(self):
        label = frame_with(4)
        label[40:90, 20:40] = 1
        filtered, removed = filter_center_lane_off_surface(label, min_support_ratio=0.0)
        self.assertIs(filtered, label)
        self.assertEqual(removed, 0)

    def test_frame_without_center_lane_avoids_copying(self):
        label = frame_with(4)
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertIs(filtered, label)
        self.assertEqual(removed, 0)

    def test_lane_drive_prefers_the_center_lane_lying_on_road(self):
        label = road_and_shortcut_frame()
        label[230:310, 150:170] = 1      # road 위
        label[230:310, 470:490] = 1      # shortcut 위
        filtered, removed = filter_center_lane_off_surface(
            label, preferred_surface=ROAD_CLASS)
        self.assertEqual(removed, 80 * 20)
        self.assertTrue(np.all(filtered[230:310, 150:170] == 1))
        # 떨어져 나간 성분은 어느 면 위였든 background 가 된다. 원래 무엇이었는지
        # 모델이 알려 준 적이 없으므로 주행면으로 되메우지 않는다.
        self.assertTrue(np.all(filtered[230:310, 470:490] == 0))

    def test_shortcut_mode_prefers_the_center_lane_lying_on_shortcut(self):
        label = road_and_shortcut_frame()
        label[230:310, 150:170] = 1
        label[230:310, 470:490] = 1
        filtered, removed = filter_center_lane_off_surface(
            label, preferred_surface=SHORTCUT_CLASS)
        self.assertEqual(removed, 80 * 20)
        self.assertTrue(np.all(filtered[230:310, 470:490] == 1))
        self.assertTrue(np.all(filtered[230:310, 150:170] == 0))

    def test_falls_back_to_any_surface_when_the_preferred_one_has_no_lane(self):
        """모드 전환 구간: 우선 면 위에 중앙선이 없으면 다른 주행면 것을 살린다."""
        label = road_and_shortcut_frame()
        label[230:310, 470:490] = 1      # shortcut 위에만 있다
        filtered, removed = filter_center_lane_off_surface(
            label, preferred_surface=ROAD_CLASS)
        self.assertEqual(removed, 0)
        self.assertTrue(np.all(filtered[230:310, 470:490] == 1))

    def test_fallback_still_drops_lanes_off_every_surface(self):
        """우선 면에 중앙선이 없어도 트랙 밖 유령까지 살려 주지는 않는다."""
        label = road_and_shortcut_frame()
        label[40:90, 20:40] = 1
        filtered, removed = filter_center_lane_off_surface(
            label, preferred_surface=ROAD_CLASS)
        self.assertEqual(removed, 50 * 20)
        self.assertEqual(np.count_nonzero(filtered == 1), 0)

    def test_without_a_preferred_surface_both_surfaces_are_equal(self):
        label = road_and_shortcut_frame()
        label[230:310, 150:170] = 1
        label[230:310, 470:490] = 1
        filtered, removed = filter_center_lane_off_surface(label)
        self.assertEqual(removed, 0)
        self.assertEqual(np.count_nonzero(filtered == 1), 2 * 80 * 20)

    def test_preferred_surface_still_removes_the_phantom_outside_the_track(self):
        label = road_and_shortcut_frame()
        label[230:310, 150:170] = 1      # road 위 진짜 중앙선
        label[40:90, 20:40] = 1          # 트랙 밖 유령
        filtered, removed = filter_center_lane_off_surface(
            label, preferred_surface=ROAD_CLASS)
        self.assertEqual(removed, 50 * 20)
        self.assertEqual(np.count_nonzero(filtered == 1), 80 * 20)

    def test_rejects_non_2d_label(self):
        with self.assertRaises(ValueError):
            filter_center_lane_off_surface(np.zeros((4, 4, 3), dtype=np.uint8))


if __name__ == '__main__':
    unittest.main()
