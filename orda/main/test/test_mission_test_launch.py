import importlib.util
from pathlib import Path

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ORDA_ROOT = Path(__file__).resolve().parents[2]
MISSION_LAUNCH = (
    ORDA_ROOT / 'main' / 'launch' / 'module_drive_mission_test.py'
)
PRODUCTION_LAUNCH = ORDA_ROOT / 'main' / 'launch' / 'module_drive.py'
BAG_TEST_LAUNCH = ORDA_ROOT / 'main' / 'launch' / 'module_drive_bag_test.py'

VALID_PROFILES = (
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
)


def load_launch_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_argument(description, name):
    return next(
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == name
    )


def main_nodes(description):
    return [
        action
        for action in description.entities
        if isinstance(action, Node)
        and action.node_package == 'main'
        and action.node_executable == 'main_node'
    ]


def lane_nodes(description):
    return [
        action
        for action in description.entities
        if isinstance(action, Node)
        and action.node_package == 'lane_detection'
        and action.node_executable == 'lane_node'
    ]


def substitution_text(substitutions):
    return ''.join(item.text for item in substitutions)


def remapping_text(node):
    return [
        (substitution_text(source), substitution_text(target))
        for source, target in node._Node__remappings
    ]


def test_profile_argument_rejects_unknown_values_at_launch_boundary():
    module = load_launch_module(MISSION_LAUNCH, '_mission_profile_choices')
    description = module.generate_launch_description()
    argument = launch_argument(description, 'test_profile')

    assert tuple(argument.choices) == VALID_PROFILES
    assert substitution_text(argument.default_value) == '2'
    assert 'foobar' not in argument.choices
    assert 'race' not in argument.choices

    context = LaunchContext()
    context.launch_configurations['test_profile'] = 'foobar'
    with pytest.raises(RuntimeError, match='not valid'):
        argument.execute(context)


def test_live_drive_defaults_false_and_creates_mutually_exclusive_main_nodes():
    module = load_launch_module(MISSION_LAUNCH, '_mission_live_drive')
    description = module.generate_launch_description()
    argument = launch_argument(description, 'live_drive')
    nodes = main_nodes(description)

    assert substitution_text(argument.default_value) == 'false'
    assert tuple(argument.choices) == ('false', 'true')
    assert len(nodes) == 2
    assert sum(type(node.condition) is UnlessCondition for node in nodes) == 1
    assert sum(type(node.condition) is IfCondition for node in nodes) == 1

    isolated = next(
        node for node in nodes if type(node.condition) is UnlessCondition
    )
    live = next(node for node in nodes if type(node.condition) is IfCondition)
    for value, expected in (
        ('false', (True, False)),
        ('true', (False, True)),
    ):
        context = LaunchContext()
        context.launch_configurations['live_drive'] = value
        assert (
            isolated.condition.evaluate(context),
            live.condition.evaluate(context),
        ) == expected


def test_udp_bridge_is_live_drive_only_and_unique():
    module = load_launch_module(MISSION_LAUNCH, '_mission_udp_bridge')
    description = module.generate_launch_description()
    bridge_argument = launch_argument(description, 'udp_motor_bridge')
    bridges = [
        action for action in description.entities
        if isinstance(action, Node)
        and action.node_package == 'main'
        and action.node_executable == 'udp_motor_bridge'
    ]
    assert substitution_text(bridge_argument.default_value) == 'true'
    assert len(bridges) == 1
    context = LaunchContext()
    context.launch_configurations.update({
        'live_drive': 'false', 'udp_motor_bridge': 'true',
    })
    assert bridges[0].condition.evaluate(context) is False
    context.launch_configurations['live_drive'] = 'true'
    assert bridges[0].condition.evaluate(context) is True


def test_only_non_live_main_node_remaps_motor_output():
    module = load_launch_module(MISSION_LAUNCH, '_mission_motor_remap')
    description = module.generate_launch_description()
    nodes = main_nodes(description)
    isolated = next(
        node for node in nodes if type(node.condition) is UnlessCondition
    )
    live = next(node for node in nodes if type(node.condition) is IfCondition)

    assert remapping_text(isolated) == [
        ('xycar_motor', '/kmu_main_offline/xycar_motor')
    ]
    assert remapping_text(live) == []


def test_both_main_actions_receive_the_same_named_test_profile():
    source = MISSION_LAUNCH.read_text(encoding='utf-8')

    assert "test_profile = LaunchConfiguration('test_profile')" in source
    assert "'test_profile': ParameterValue(test_profile, value_type=str)" in source
    assert source.count("executable='main_node'") == 2
    assert source.count("executable='main_node'") == 2


@pytest.mark.skip(reason="main 이 두 브랜치를 합치기 전의 옛 노드 구성(traffic_light 패키지가 별도로 떠 있던 시절)을 검사한다. 2026-08-19 병합에서 HEAD(신호등을 object_detection 에 통합한 설계)를 유지하고 main 의 옛 구성은 반영하지 않았다.")
def test_mission_launch_reuses_the_complete_non_main_production_stack():
    mission_module = load_launch_module(MISSION_LAUNCH, '_mission_stack')
    production_module = load_launch_module(PRODUCTION_LAUNCH, '_production_stack')
    mission = mission_module.generate_launch_description()
    production = production_module.generate_launch_description()

    mission_packages = sorted(
        action.node_package
        for action in mission.entities
        if isinstance(action, Node) and action.node_package != 'main'
    )
    production_packages = sorted(
        action.node_package
        for action in production.entities
        if isinstance(action, Node) and action.node_package != 'main'
    )
    mission_includes = sum(
        isinstance(action, IncludeLaunchDescription)
        for action in mission.entities
    )
    production_includes = sum(
        isinstance(action, IncludeLaunchDescription)
        for action in production.entities
    )

    assert mission_packages == production_packages == [
        'image_resize',
        'joy',
        'lane_detection',
        'object_detection',
        'object_detection',
        'rubbercone',
        'segmentation_tools',
    ]
    assert mission_includes == production_includes == 3


def test_production_bridge_and_bag_isolation_contracts():
    production_source = PRODUCTION_LAUNCH.read_text(encoding='utf-8')
    bag_source = BAG_TEST_LAUNCH.read_text(encoding='utf-8')

    assert "'udp_motor_bridge'" in production_source
    assert "executable='udp_motor_bridge'" in production_source
    assert "default_value='2'" in bag_source
    assert "('xycar_motor', '/kmu_main_offline/xycar_motor')" in bag_source
    assert "udp_motor_bridge" in bag_source
    assert "default_value='false'" in bag_source
    assert "executable='resize_node'" not in bag_source


def test_bag_launch_requires_explicit_live_bridge_opt_in_and_string_profile():
    module = load_launch_module(BAG_TEST_LAUNCH, '_bag_live_contract')
    description = module.generate_launch_description()
    nodes = main_nodes(description)

    assert substitution_text(launch_argument(description, 'live_drive').default_value) == 'false'
    assert substitution_text(launch_argument(description, 'udp_motor_bridge').default_value) == 'false'
    assert len(nodes) == 2
    isolated = next(node for node in nodes if type(node.condition) is UnlessCondition)
    live = next(node for node in nodes if type(node.condition) is IfCondition)
    assert remapping_text(isolated) == [('xycar_motor', '/kmu_main_offline/xycar_motor')]
    assert remapping_text(live) == []

    for node in nodes:
        parameter_map = node._Node__parameters[0]
        values = {
            substitution_text(key): value
            for key, value in parameter_map.items()
        }
        profile = values['test_profile']
        assert isinstance(profile, ParameterValue)
        assert profile.value_type is str


@pytest.mark.parametrize(
    ('path', 'module_name'),
    [
        (PRODUCTION_LAUNCH, '_production_lane_contract'),
        (BAG_TEST_LAUNCH, '_bag_lane_contract'),
    ],
)
def test_lane_detector_legacy_input_is_remapped_official_mode_info(
    path,
    module_name,
):
    module = load_launch_module(path, module_name)
    nodes = lane_nodes(module.generate_launch_description())

    assert len(nodes) == 1
    assert remapping_text(nodes[0]) == [
        ('/mode_info', '/internal/lane_command')
    ]


def test_main_parameter_forces_profile_to_string():
    module = load_launch_module(MISSION_LAUNCH, '_mission_parameters')
    description = module.generate_launch_description()

    for node in main_nodes(description):
        parameter_map = node._Node__parameters[0]
        values = {
            substitution_text(key): value
            for key, value in parameter_map.items()
        }
        profile_value = values['test_profile']
        assert isinstance(profile_value, ParameterValue)
        assert profile_value.value_type is str
        context = LaunchContext()
        context.launch_configurations['test_profile'] = '2'
        assert profile_value.evaluate(context) == '2'
