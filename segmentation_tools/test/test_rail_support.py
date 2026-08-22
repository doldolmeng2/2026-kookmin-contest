"""바깥 실선(left_solid/right_solid)의 주행면 검증을 확인한다.

중앙선과 같은 근거(주행면에 붙어 있는가)를 쓰지만 임계값이 다르다. 실선은
바깥쪽이 정당하게 background 라 테두리의 road 비율이 구조적으로 낮다 — 아래
scene() 에서 진짜 실선이 0.44 근처로 나오므로, 중앙선의 0.5 를 그대로 쓰면
진짜를 지운다. 그 회귀를 막는 것이 이 파일의 목적이다.
"""
import unittest

import numpy as np

from segmentation_tools.core import (
    BACKGROUND_CLASS,
    CENTER_LANE_MIN_SUPPORT_RATIO,
    DRIVABLE_SURFACE_CLASSES,
    LEFT_SOLID_CLASS,
    RAIL_MIN_SUPPORT_RATIO,
    RAIL_SUPPORT_RADIUS,
    RIGHT_SOLID_CLASS,
    filter_rails_off_surface,
    lane_components,
)


def scene():
    """road 양옆에 진짜 실선, 트랙 밖에 가짜 실선을 하나씩 둔 라벨."""
    label = np.zeros((120, 200), np.uint8)
    label[20:100, 60:140] = DRIVABLE_SURFACE_CLASSES[0]   # road
    label[20:100, 58:60] = LEFT_SOLID_CLASS               # 진짜 (안쪽이 road)
    label[20:100, 140:142] = RIGHT_SOLID_CLASS            # 진짜
    label[40:60, 10:13] = LEFT_SOLID_CLASS                # 가짜 (사방 background)
    label[30:70, 175:178] = RIGHT_SOLID_CLASS             # 가짜
    return label


class RailSupportTest(unittest.TestCase):
    def test_real_rail_support_sits_between_the_two_thresholds(self):
        """진짜 실선의 road 비율이 레일 임계 위, 중앙선 임계 아래여야 한다.

        이 관계가 깨지면 레일 임계를 중앙선 값으로 되돌려도 테스트가 통과해
        버린다. 임계를 둘로 나눈 이유 자체를 고정한다.
        """
        for lane_class in (LEFT_SOLID_CLASS, RIGHT_SOLID_CLASS):
            components = lane_components(
                scene(), lane_class, RAIL_SUPPORT_RADIUS,
                DRIVABLE_SURFACE_CLASSES, None)
            real = max(components, key=lambda component: component[2])
            self.assertGreater(real[3], RAIL_MIN_SUPPORT_RATIO)
            self.assertLess(real[3], CENTER_LANE_MIN_SUPPORT_RATIO)

    def test_off_surface_rails_are_removed_and_real_ones_kept(self):
        label = scene()
        filtered, removed = filter_rails_off_surface(label)
        self.assertGreater(removed, 0)
        # 진짜는 남는다.
        self.assertEqual(filtered[50, 58], LEFT_SOLID_CLASS)
        self.assertEqual(filtered[50, 140], RIGHT_SOLID_CLASS)
        # 가짜는 background 가 된다.
        self.assertEqual(filtered[50, 11], BACKGROUND_CLASS)
        self.assertEqual(filtered[50, 176], BACKGROUND_CLASS)
        # road 는 건드리지 않는다.
        self.assertEqual(filtered[50, 100], DRIVABLE_SURFACE_CLASSES[0])

    def test_both_rails_survive_together(self):
        """레일에는 winner-take-all 을 쓰면 안 된다.

        중앙선 규칙을 그대로 쓰면 우선 면 위의 성분 하나가 나머지를 지운다.
        좌우 실선은 서로 독립이라 둘 다 살아남아야 한다.
        """
        filtered, _ = filter_rails_off_surface(scene())
        self.assertTrue((filtered == LEFT_SOLID_CLASS).any())
        self.assertTrue((filtered == RIGHT_SOLID_CLASS).any())

    def test_disabled_by_zero_returns_input_untouched(self):
        label = scene()
        filtered, removed = filter_rails_off_surface(label, min_support_ratio=0.0)
        self.assertEqual(removed, 0)
        self.assertIs(filtered, label)

    def test_center_lane_is_not_touched(self):
        """레일 필터는 center_lane 을 건드리지 않는다."""
        label = scene()
        label[20:100, 98:102] = 1
        filtered, _ = filter_rails_off_surface(label)
        self.assertEqual(int((filtered == 1).sum()), int((label == 1).sum()))


if __name__ == '__main__':
    unittest.main()
