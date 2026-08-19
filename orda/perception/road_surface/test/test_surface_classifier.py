from types import SimpleNamespace

import numpy as np
import pytest

from road_surface.surface_classifier import (
    RoadSurface,
    SurfaceThresholds,
    class_map_from_image,
    classify_surface,
)


def thresholds():
    return SurfaceThresholds(
        road_min_ratio=0.25,
        shortcut_min_ratio=0.20,
        road_min_component_px=8,
        shortcut_min_component_px=8,
    )


def test_class_four_produces_normal_road():
    class_map = np.zeros((10, 10), dtype=np.uint8)
    class_map[5:, :] = 4
    surface, evidence = classify_surface(class_map, thresholds())
    assert surface is RoadSurface.NORMAL_ROAD
    assert evidence.road_ratio == pytest.approx(0.5)


def test_class_five_has_priority_when_both_candidates_are_sufficient():
    class_map = np.zeros((10, 10), dtype=np.uint8)
    class_map[:5, :] = 4
    class_map[5:, :] = 5
    surface, _ = classify_surface(class_map, thresholds())
    assert surface is RoadSurface.SHORTCUT


def test_insufficient_ratio_or_component_is_unknown():
    class_map = np.zeros((10, 10), dtype=np.uint8)
    class_map[::2, ::2] = 5
    surface, evidence = classify_surface(class_map, thresholds())
    assert evidence.shortcut_ratio >= 0.20
    assert evidence.shortcut_largest_component_px < 8
    assert surface is RoadSurface.UNKNOWN


def test_roi_excludes_labels_outside_configured_region():
    class_map = np.full((10, 10), 5, dtype=np.uint8)
    class_map[5:, :] = 0
    config = SurfaceThresholds(
        road_min_ratio=0.1,
        shortcut_min_ratio=0.1,
        road_min_component_px=1,
        shortcut_min_component_px=1,
        roi_top=0.5,
    )
    surface, _ = classify_surface(class_map, config)
    assert surface is RoadSurface.UNKNOWN


def test_ros_image_decoder_honors_step_padding():
    message = SimpleNamespace(
        height=2,
        width=3,
        encoding="mono8",
        is_bigendian=0,
        step=4,
        data=bytes([0, 1, 2, 99, 3, 4, 5, 99]),
    )
    decoded = class_map_from_image(message)
    assert decoded.tolist() == [[0, 1, 2], [3, 4, 5]]


@pytest.mark.parametrize(
    "message",
    [
        SimpleNamespace(
            height=0,
            width=3,
            encoding="mono8",
            is_bigendian=0,
            step=3,
            data=b"",
        ),
        SimpleNamespace(
            height=2,
            width=3,
            encoding="rgb8",
            is_bigendian=0,
            step=9,
            data=bytes(18),
        ),
        SimpleNamespace(
            height=2,
            width=3,
            encoding="mono8",
            is_bigendian=0,
            step=3,
            data=bytes(5),
        ),
    ],
)
def test_malformed_ros_image_is_rejected(message):
    with pytest.raises(ValueError):
        class_map_from_image(message)
