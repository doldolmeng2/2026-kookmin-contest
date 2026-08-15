import math

import pytest

from manual_drive import joystic
from manual_drive.joystic import (
    ManualDriveCommand,
    ManualDriveConfig,
    ManualDriveGate,
    STOP_COMMAND,
)


def _gate(**overrides):
    return ManualDriveGate(ManualDriveConfig(deadman_button=2, **overrides))


def test_axis_mapping_matches_manual_drive_contract():
    gate = _gate(max_abs_speed=50.0, max_abs_steering=100.0)

    command = gate.record_joy(
        axes=[0.2, 0.0, 0.0, 0.0, 0.1],
        buttons=[0, 0, 1],
        received_at=1.0,
    )

    assert command == ManualDriveCommand(steering=-20.0, speed=5.0)


def test_command_is_clamped_to_configured_vehicle_limits():
    gate = _gate()

    command = gate.record_joy(
        axes=[-1.0, 0.0, 0.0, 0.0, -1.0],
        buttons=[0, 0, 1],
        received_at=1.0,
    )

    assert command == ManualDriveCommand(steering=45.0, speed=-7.5)


@pytest.mark.parametrize(
    ('buttons', 'expected'),
    [
        ([0, 0, 0], STOP_COMMAND),
        ([0, 0, 1], ManualDriveCommand(steering=-10.0, speed=5.0)),
    ],
)
def test_deadman_gates_nonzero_commands(buttons, expected):
    gate = _gate(max_abs_speed=50.0)

    assert gate.record_joy(
        axes=[0.1, 0.0, 0.0, 0.0, 0.1],
        buttons=buttons,
        received_at=1.0,
    ) == expected


def test_deadman_release_immediately_returns_stop():
    gate = _gate()
    axes = [0.1, 0.0, 0.0, 0.0, 0.1]
    gate.record_joy(axes, [0, 0, 1], received_at=1.0)

    assert gate.record_joy(
        axes,
        [0, 0, 0],
        received_at=1.01,
    ) == STOP_COMMAND


def test_stale_joy_never_holds_previous_command():
    gate = _gate(joy_timeout_s=0.25)
    gate.record_joy(
        [0.1, 0.0, 0.0, 0.0, 0.1],
        [0, 0, 1],
        received_at=10.0,
    )

    assert gate.command_at(10.25) != STOP_COMMAND
    assert gate.command_at(10.251) == STOP_COMMAND
    assert gate.command_at(10.252) == STOP_COMMAND


@pytest.mark.parametrize(
    ('axes', 'buttons'),
    [
        ([], [0, 0, 1]),
        ([0.1, 0.0, 0.0, 0.0, 0.1], []),
        ([math.nan, 0.0, 0.0, 0.0, 0.1], [0, 0, 1]),
        ([0.1, 0.0, 0.0, 0.0, math.inf], [0, 0, 1]),
    ],
)
def test_malformed_or_nonfinite_joy_returns_stop(axes, buttons):
    assert _gate().record_joy(axes, buttons, received_at=1.0) == STOP_COMMAND


def test_startup_and_unconfigured_deadman_are_stop_only():
    unconfigured = ManualDriveGate(ManualDriveConfig(deadman_button=-1))

    assert unconfigured.command_at(0.0) == STOP_COMMAND
    assert unconfigured.record_joy(
        [1.0, 0.0, 0.0, 0.0, 1.0],
        [1] * 12,
        received_at=0.0,
    ) == STOP_COMMAND


@pytest.mark.parametrize(
    'kwargs',
    [
        {'steering_axis': -1},
        {'speed_axis': True},
        {'deadman_button': -2},
        {'max_abs_steering': 0.0},
        {'max_abs_speed': math.inf},
        {'joy_timeout_s': -0.1},
        {'publish_rate_hz': math.nan},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ManualDriveConfig(**kwargs)


def test_keyboard_interrupt_attempts_stop_before_cleanup(monkeypatch):
    events = []

    class FakeNode:
        def publish_stop(self):
            events.append('publish_stop')

        def destroy_node(self):
            events.append('destroy_node')

    fake_node = FakeNode()
    monkeypatch.setattr(joystic.rclpy, 'init', lambda args=None: None)
    monkeypatch.setattr(joystic, 'JoyToMotor', lambda: fake_node)
    monkeypatch.setattr(
        joystic.rclpy,
        'spin',
        lambda node: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(joystic.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        joystic.rclpy,
        'shutdown',
        lambda: events.append('shutdown'),
    )

    joystic.main()

    assert events == ['publish_stop', 'destroy_node', 'shutdown']
