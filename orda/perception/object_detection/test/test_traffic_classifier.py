import numpy as np
import pytest

from object_detection.traffic_classifier import (
    TRAFFIC_CLASS_NAMES,
    classifier_blob,
    classify_current_frame_candidate,
    crop_detection,
    parse_class_names,
    traffic_signal_from_output,
    validate_classifier_classes,
    validate_detector_classes,
)
from object_detection.yolo_runtime import Detection


def test_detector_and_classifier_metadata_contracts():
    detector = parse_class_names(
        "{0: 'red_car', 1: 'green_car', 2: 'traffic_a', "
        "3: 'traffic_b', 4: 'traffic_c', 5: 'traffic_d'}"
    )
    validate_detector_classes(detector)
    validate_classifier_classes(TRAFFIC_CLASS_NAMES)


def test_wrong_single_class_detector_is_rejected():
    with pytest.raises(ValueError, match="class 0"):
        validate_detector_classes({0: "obstacle_car"})


@pytest.mark.parametrize(
    ("scores", "expected"),
    [
        ([0.9, 0.05, 0.03, 0.02], 2),
        ([0.05, 0.9, 0.03, 0.02], 3),
        ([0.05, 0.03, 0.9, 0.02], 1),
        ([0.05, 0.03, 0.02, 0.9], 1),
    ],
)
def test_classifier_mapping(scores, expected):
    assert traffic_signal_from_output(np.asarray([scores]), 0.6) == expected


def test_uncertain_classifier_output_maps_to_no_signal():
    assert traffic_signal_from_output(np.asarray([[0.26, 0.25, 0.25, 0.24]]), 0.6) == 0


def test_malformed_classifier_output_is_not_accepted():
    with pytest.raises(ValueError, match="four finite"):
        traffic_signal_from_output(np.asarray([[0.5, np.nan, 0.5]]), 0.6)


def test_crop_is_copied_from_only_the_current_frame():
    first = np.full((20, 30, 3), 7, dtype=np.uint8)
    second = np.full((20, 30, 3), 99, dtype=np.uint8)
    detection = Detection(2, 0.9, 5, 6, 10, 8)

    first_crop = crop_detection(first, detection)
    second_crop = crop_detection(second, detection)
    second[:, :, :] = 0

    assert np.all(first_crop == 7)
    assert np.all(second_crop == 99)


def test_classifier_blob_has_expected_shape_and_dtype():
    blob = classifier_blob(np.zeros((12, 8, 3), dtype=np.uint8), 224, 224)
    assert blob.shape == (1, 3, 224, 224)
    assert blob.dtype == np.float32


class FakeClassifier:
    def __init__(self, output):
        self.output = output
        self.calls = []

    def run(self, output_names, inputs):
        self.calls.append((output_names, inputs))
        return [self.output]


def test_no_detector_candidate_maps_to_zero_without_classifier_call():
    session = FakeClassifier(np.asarray([[1.0, 0.0, 0.0, 0.0]]))
    signal = classify_current_frame_candidate(
        np.zeros((20, 20, 3), dtype=np.uint8),
        None,
        session,
        "images",
        224,
        224,
        0.6,
    )
    assert signal == 0
    assert session.calls == []


def test_two_frames_never_reuse_the_previous_candidate_crop():
    session = FakeClassifier(np.asarray([[0.0, 1.0, 0.0, 0.0]]))
    first_signal = classify_current_frame_candidate(
        np.full((20, 20, 3), 20, dtype=np.uint8),
        Detection(2, 0.9, 2, 2, 8, 8),
        session,
        "images",
        16,
        16,
        0.6,
    )
    second_signal = classify_current_frame_candidate(
        np.full((20, 20, 3), 200, dtype=np.uint8),
        None,
        session,
        "images",
        16,
        16,
        0.6,
    )
    assert first_signal == 3
    assert second_signal == 0
    assert len(session.calls) == 1
