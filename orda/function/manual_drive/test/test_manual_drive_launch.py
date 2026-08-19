import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


LAUNCH_PATH = Path(__file__).parents[1] / 'launch' / 'manual_drive.py'
LEGACY_LAUNCH_PATH = (
    Path(__file__).parents[1] / 'launch' / 'manual_drive.launch.py'
)


def _description():
    spec = importlib.util.spec_from_file_location('manual_drive_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _load_description(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _entities(entity_type):
    return [
        entity for entity in _description().entities
        if isinstance(entity, entity_type)
    ]


def _arguments():
    return {
        action.name: action
        for action in _entities(DeclareLaunchArgument)
    }


def _manual_nodes():
    return [
        node for node in _entities(Node)
        if node.node_package == 'manual_drive'
    ]


def _substitution_text(substitutions):
    return ''.join(item.text for item in substitutions)


def _remapping_text(node):
    return [
        (_substitution_text(source), _substitution_text(target))
        for source, target in node._Node__remappings
    ]


def _apply_argument(context, name, value=None):
    argument = _arguments()[name]
    if value is not None:
        context.launch_configurations[name] = value
    argument.execute(context)


def test_live_drive_defaults_false_and_rejects_invalid_boolean():
    context = LaunchContext()
    _apply_argument(context, 'live_drive')
    assert context.launch_configurations['live_drive'] == 'false'

    with pytest.raises(RuntimeError):
        _apply_argument(LaunchContext(), 'live_drive', 'yes')


def test_only_one_manual_node_is_enabled_for_each_live_drive_value():
    nodes = _manual_nodes()
    assert len(nodes) == 2
    assert {type(node.condition) for node in nodes} == {
        IfCondition,
        UnlessCondition,
    }

    for value in ('false', 'true'):
        context = LaunchContext()
        context.launch_configurations['live_drive'] = value
        enabled = [node for node in nodes if node.condition.evaluate(context)]
        assert len(enabled) == 1


def test_isolated_node_remaps_motor_and_live_node_does_not():
    nodes = _manual_nodes()
    isolated = next(
        node for node in nodes if type(node.condition) is UnlessCondition
    )
    live = next(node for node in nodes if type(node.condition) is IfCondition)

    assert _remapping_text(isolated) == [
        ('xycar_motor', '/kmu_main_offline/xycar_motor')
    ]
    assert _remapping_text(live) == []


def test_launch_contains_only_joy_and_manual_nodes():
    nodes = _entities(Node)
    packages = [node.node_package for node in nodes]

    assert packages.count('joy') == 1
    assert packages.count('manual_drive') == 2
    assert set(packages) == {'joy', 'manual_drive'}


def test_deadman_default_is_stop_only_and_all_runtime_parameters_are_exposed():
    arguments = _arguments()
    expected = {
        'steering_axis',
        'steering_scale',
        'speed_axis',
        'speed_scale',
        'deadman_button',
        'max_abs_steering',
        'max_abs_speed',
        'joy_timeout_s',
        'publish_rate_hz',
    }
    assert expected <= set(arguments)

    context = LaunchContext()
    _apply_argument(context, 'deadman_button')
    assert context.launch_configurations['deadman_button'] == '-1'


def test_legacy_launch_entry_point_delegates_to_safe_default():
    description = _load_description(LEGACY_LAUNCH_PATH, 'legacy_manual_launch')
    live_argument = next(
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'live_drive'
    )
    manual_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_package == 'manual_drive'
    ]
    isolated = next(
        node for node in manual_nodes
        if type(node.condition) is UnlessCondition
    )

    context = LaunchContext()
    live_argument.execute(context)
    assert context.launch_configurations['live_drive'] == 'false'
    assert _remapping_text(isolated) == [
        ('xycar_motor', '/kmu_main_offline/xycar_motor')
    ]
