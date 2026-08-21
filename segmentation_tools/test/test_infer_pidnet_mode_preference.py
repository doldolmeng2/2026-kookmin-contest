"""center_lane 우선 주행면이 /mode_info 를 따라가는지 확인한다."""
from types import SimpleNamespace
import unittest

from segmentation_tools.core import ROAD_CLASS, SHORTCUT_CLASS
from segmentation_tools.infer_pidnet import (PREFERRED_SURFACE_BY_MODE,
                                             PIDNetInferenceNode)


def stub_node(preferred=None):
    """mode_callback 이 건드리는 것만 갖춘 가짜 노드."""
    return SimpleNamespace(
        runner=SimpleNamespace(preferred_surface=preferred),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )


class ModePreferenceTest(unittest.TestCase):
    def test_mode_info_contract_maps_to_surfaces(self):
        self.assertEqual(PREFERRED_SURFACE_BY_MODE[1], ROAD_CLASS)      # LANE_DRIVE
        self.assertEqual(PREFERRED_SURFACE_BY_MODE[5], SHORTCUT_CLASS)  # SHORTCUT

    def test_lane_drive_selects_road(self):
        node = stub_node()
        PIDNetInferenceNode.mode_callback(node, SimpleNamespace(data=1))
        self.assertEqual(node.runner.preferred_surface, ROAD_CLASS)

    def test_shortcut_selects_shortcut(self):
        node = stub_node()
        PIDNetInferenceNode.mode_callback(node, SimpleNamespace(data=5))
        self.assertEqual(node.runner.preferred_surface, SHORTCUT_CLASS)

    def test_other_modes_clear_the_preference(self):
        """라바콘/회피/추월 구간은 우선 면 없이 road·shortcut 을 모두 받는다."""
        node = stub_node(ROAD_CLASS)
        PIDNetInferenceNode.mode_callback(node, SimpleNamespace(data=2))
        self.assertIsNone(node.runner.preferred_surface)

    def test_repeated_mode_message_keeps_the_same_preference(self):
        node = stub_node(SHORTCUT_CLASS)
        PIDNetInferenceNode.mode_callback(node, SimpleNamespace(data=5))
        self.assertEqual(node.runner.preferred_surface, SHORTCUT_CLASS)


if __name__ == '__main__':
    unittest.main()
