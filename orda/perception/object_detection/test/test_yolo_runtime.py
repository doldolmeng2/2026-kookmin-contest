import numpy as np
import pytest

from object_detection.yolo_runtime import (
    Detection,
    closest_detection,
    closest_detection_for_classes,
    decode_detections,
    detection_slot,
    normalize_class_ids,
    prediction_rows,
)


def test_normalizes_current_one_class_export():
    output = np.zeros((1, 5, 8400), dtype=np.float32)
    assert prediction_rows(output).shape == (8400, 5)


def test_decodes_dynamic_multiclass_export_and_filters_signal_class():
    # Three anchors, three classes. Class 2 represents a traffic signal and is
    # excluded from the object detector by allowed_class_ids.
    output = np.zeros((1, 7, 3), dtype=np.float32)
    output[0, :4, 0] = [100, 120, 40, 30]
    output[0, 4:, 0] = [0.9, 0.1, 0.0]
    output[0, :4, 1] = [300, 250, 80, 70]
    output[0, 4:, 1] = [0.1, 0.95, 0.0]
    output[0, :4, 2] = [400, 300, 100, 100]
    output[0, 4:, 2] = [0.0, 0.0, 0.99]

    detections = decode_detections(
        output,
        image_shape=(640, 640),
        scale=1.0,
        pad_x=0.0,
        pad_y=0.0,
        confidence_threshold=0.5,
        nms_threshold=0.4,
        allowed_class_ids={0, 1},
    )
    assert {item.class_id for item in detections} == {0, 1}
    assert closest_detection(detections).class_id == 1


def test_accepts_transposed_rows():
    output = np.array([[20, 20, 16, 16, 0.8]], dtype=np.float32)
    detections = decode_detections(
        output,
        image_shape=(40, 40),
        scale=1.0,
        pad_x=0.0,
        pad_y=0.0,
        confidence_threshold=0.5,
        nms_threshold=0.4,
        allowed_class_ids={0},
    )
    assert len(detections) == 1
    assert detections[0].area == 256


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
    """한 프레임에 red_car(4)와 green_car(0)이 같이 보이는 경우.

    단일 슬롯이었을 때는 면적이 큰 쪽만 살아남아, /object_info 의 고정차량
    위치와 방해차량 위치 중 하나가 항상 0 이었다.
    """
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


def test_detection_slot_rejects_an_unknown_semantic_type():
    with pytest.raises(ValueError, match="fixed"):
        detection_slot(None, -1)
