import numpy as np
import pytest

from object_detection.yolo_runtime import (
    Detection,
    LatestFrameRateLimiter,
    closest_detection_for_classes,
    detection_slot,
    normalize_class_ids,
)


def test_normalize_class_ids_deduplicates_integer_values():
    assert normalize_class_ids([2, 1, 2]) == {1, 2}


def test_normalize_class_ids_rejects_negative_values():
    with pytest.raises(ValueError, match="non-negative"):
        normalize_class_ids([-1, 2])


def test_closest_detection_for_classes_filters_before_area_selection():
    detections = [
        Detection(0, 0.9, 0, 0, 100, 100),
        Detection(2, 0.8, 0, 0, 20, 30),
        Detection(3, 0.7, 0, 0, 30, 30),
    ]
    assert closest_detection_for_classes(detections, [2, 3]) == detections[2]
    assert closest_detection_for_classes(detections, [5]) is None


def test_closest_detection_for_classes_accepts_numpy_integer_ids():
    detection = Detection(2, 0.8, 0, 0, 20, 30)
    assert closest_detection_for_classes(
        [detection], np.asarray([2], dtype=np.int64)
    ) == detection


def test_fixed_and_moving_slots_preserve_both_classes_from_one_frame():
    detections = [
        Detection(4, 0.8, 0, 0, 30, 30),
        Detection(0, 0.9, 2, 3, 25, 20),
    ]
    payload = (
        detection_slot(closest_detection_for_classes(detections, [4]), 0)
        + detection_slot(closest_detection_for_classes(detections, [0]), 1)
    )

    assert len(payload) == 20
    assert payload[0:2] == [1.0, 0.0]
    assert payload[10:12] == [1.0, 1.0]


def test_empty_slots_keep_their_semantic_types():
    assert detection_slot(None, 0) == [0.0, 0.0] + [0.0] * 8
    assert detection_slot(None, 1) == [0.0, 1.0] + [0.0] * 8


def test_unlimited_rate_limiter_returns_every_frame_immediately():
    limiter = LatestFrameRateLimiter(0.0)
    assert limiter.offer("first", 10.0) == "first"
    assert limiter.offer("second", 10.0) == "second"
    assert not limiter.has_pending


def test_limited_rate_limiter_replaces_pending_frame_with_latest():
    limiter = LatestFrameRateLimiter(10.0)
    assert limiter.offer("first", 1.0) == "first"
    assert limiter.offer("old", 1.02) is None
    assert limiter.offer("latest", 1.04) is None
    assert limiter.pop_due(1.099) is None
    assert limiter.pop_due(1.10) == "latest"
    assert not limiter.has_pending


def test_limited_rate_limiter_preserves_selected_object_identity():
    limiter = LatestFrameRateLimiter(5.0)
    first = object()
    latest = object()
    assert limiter.offer(first, 0.0) is first
    assert limiter.offer(latest, 0.1) is None
    assert limiter.pop_due(0.2) is latest
