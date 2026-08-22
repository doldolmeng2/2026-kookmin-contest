import pytest

from object_detection.traffic_classifier import (
    DETECTOR_CLASS_NAMES,
    DETECTOR_TRAFFIC_TO_SIGNAL_INDEX,
    SIGNAL_INDEX_TO_TRAFFIC_STATE,
    detector_object_type,
    detector_traffic_box_values,
    detector_traffic_to_signal_index,
    detector_traffic_to_state,
    parse_class_names,
    validate_detector_classes,
)


def test_train10_detector_metadata_contract():
    names = parse_class_names(str(DETECTOR_CLASS_NAMES))
    assert names == {
        0: "green_car",
        1: "green_light",
        2: "left_green_light",
        3: "orange_light",
        4: "red_car",
        5: "red_light",
        6: "traffic",
    }
    validate_detector_classes(names)


def test_wrong_detector_contract_is_rejected():
    wrong = dict(DETECTOR_CLASS_NAMES)
    wrong[1] = "traffic"
    with pytest.raises(ValueError, match="train-10"):
        validate_detector_classes(wrong)


@pytest.mark.parametrize(
    ("detector_class", "signal_index", "traffic_state"),
    [
        (1, 0, 2),
        (2, 1, 3),
        (3, 2, 1),
        (5, 3, 1),
    ],
)
def test_detector_traffic_classes_map_directly(
    detector_class, signal_index, traffic_state
):
    assert detector_traffic_to_signal_index(detector_class) == signal_index
    assert detector_traffic_to_state(detector_class) == traffic_state
    assert SIGNAL_INDEX_TO_TRAFFIC_STATE[signal_index] == traffic_state


def test_generic_traffic_class_is_not_state_evidence():
    assert 6 not in DETECTOR_TRAFFIC_TO_SIGNAL_INDEX
    assert detector_traffic_to_signal_index(6) is None
    assert detector_traffic_to_state(6) is None
    assert detector_traffic_box_values(6, 0.99, 1, 2, 3, 4) is None


def test_red_car_remains_fixed_and_green_car_remains_moving():
    assert detector_object_type(4) == 0
    assert detector_object_type(0) == 1


def test_traffic_box_records_only_use_mapped_signal_indices():
    records = [
        detector_traffic_box_values(class_id, 0.75, 10, 20, 30, 40)
        for class_id in (1, 2, 3, 5, 6)
    ]
    published = [record for record in records if record is not None]

    assert [record[0] for record in published] == [0.0, 1.0, 2.0, 3.0]
    assert all(
        record[1:] == (0.75, 10.0, 20.0, 30.0, 40.0)
        for record in published
    )
