from types import SimpleNamespace

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from road_surface.road_surface_node import (
    FULL_LAP_THRESHOLDS,
    RoadSurfaceNode,
    sensor_qos,
)
from road_surface.surface_classifier import SurfaceThresholds


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class Harness:
    def __init__(self):
        self.thresholds = SurfaceThresholds(
            road_min_ratio=0.25,
            shortcut_min_ratio=0.25,
            road_min_component_px=4,
            shortcut_min_component_px=4,
        )
        self.publisher = Publisher()
        self.logger = Logger()

    def get_logger(self):
        return self.logger


def image_message(data, *, height=2, width=4, step=4):
    return SimpleNamespace(
        height=height,
        width=width,
        encoding="mono8",
        is_bigendian=0,
        step=step,
        data=bytes(data),
    )


def test_sensor_qos_is_best_effort_volatile_depth_one():
    qos = sensor_qos()
    assert qos.depth == 1
    assert qos.reliability is ReliabilityPolicy.BEST_EFFORT
    assert qos.durability is DurabilityPolicy.VOLATILE


def test_full_lap_threshold_contract_is_evidence_backed_trapezoid():
    assert FULL_LAP_THRESHOLDS.roi_top == 0.60
    assert FULL_LAP_THRESHOLDS.roi_top_width_ratio == 0.50
    assert FULL_LAP_THRESHOLDS.road_min_ratio == 0.90
    assert FULL_LAP_THRESHOLDS.shortcut_min_ratio == 0.30


def test_valid_camera_frame_publishes_at_most_once():
    harness = Harness()
    RoadSurfaceNode.on_class_map(
        harness,
        image_message([5, 5, 5, 5, 5, 5, 5, 5]),
    )
    assert [message.data for message in harness.publisher.messages] == [2]


def test_malformed_camera_frame_publishes_nothing():
    harness = Harness()
    RoadSurfaceNode.on_class_map(
        harness,
        image_message([5, 5, 5], height=2, width=4, step=4),
    )
    assert harness.publisher.messages == []
    assert len(harness.logger.errors) == 1
