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


def test_fixed_and_moving_representatives_survive_the_same_frame():
    detections = [
        Detection(0, 0.8, 0, 0, 20, 20),
        Detection(0, 0.7, 0, 0, 30, 30),
        Detection(1, 0.9, 0, 0, 25, 25),
    ]

    fixed = closest_detection_for_classes(detections, {0})
    moving = closest_detection_for_classes(detections, {1})
    payload = detection_slot(fixed, 0) + detection_slot(moving, 1)

    assert fixed.area == 900
    assert moving.area == 625
    assert len(payload) == 20
    assert payload[0:2] == [1.0, 0.0]
    assert payload[10:12] == [1.0, 1.0]


def test_class_id_array_supports_an_optional_empty_moving_mapping():
    assert normalize_class_ids([0]) == {0}
    assert normalize_class_ids([1, 2, 2]) == {1, 2}
    assert normalize_class_ids([]) == set()


@pytest.mark.parametrize("value", [[1.0], [-1], [True], None, "1"])
def test_class_id_array_rejects_ambiguous_or_invalid_values(value):
    with pytest.raises(ValueError):
        normalize_class_ids(value)
