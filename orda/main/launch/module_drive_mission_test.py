"""Hardware-stack mission entry launch with explicit motor-output isolation."""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MISSION_TEST_PROFILES = (
    '1',  # WAIT_GREEN
    '2',  # LANE_CENTER
    '3',  # LANE_ONE
    '4',  # LANE_TWO
    '5',  # CONE
    '6',  # FIXED
    '7',  # OVERTAKE
    '8',  # SHORTCUT
)


def _production_launch_description():
    """Load the installed production stack without modifying its source."""

    launch_path = os.path.join(
        get_package_share_directory('main'),
        'launch',
        'module_drive.py',
    )
    spec = importlib.util.spec_from_file_location(
        '_kmu_production_module_drive',
        launch_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load production launch: {launch_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _is_production_main_node(action):
    return (
        isinstance(action, Node)
        and action.node_package == 'main'
        and action.node_executable == 'main_node'
    )


def generate_launch_description():
    test_profile_arg = DeclareLaunchArgument(
        'test_profile',
        default_value='2',
        choices=MISSION_TEST_PROFILES,
        description=(
            '실차 미션 번호: 1=wait_green, 2=lane_center, 3=lane_1, '
            '4=lane_2, 5=cone, 6=fixed, 7=overtake, 8=shortcut'
        ),
    )
    live_drive_arg = DeclareLaunchArgument(
        'live_drive',
        default_value='false',
        choices=('false', 'true'),
        description=(
            'true일 때만 main_node가 실제 /xycar_motor를 publish함'
        ),
    )

    test_profile = LaunchConfiguration('test_profile')
    live_drive = LaunchConfiguration('live_drive')
    mode = LaunchConfiguration('mode')
    show_debug = LaunchConfiguration('show_debug')
    main_parameters = [{
        'mode': mode,
        'test_profile': test_profile,
        'show_debug': show_debug,
    }]

    isolated_main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=main_parameters,
        remappings=[('xycar_motor', '/mission_test/xycar_motor')],
        condition=UnlessCondition(live_drive),
    )
    live_main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=main_parameters,
        condition=IfCondition(live_drive),
    )

    production = _production_launch_description()
    entities = []
    replaced_main_nodes = 0
    for action in production.entities:
        if _is_production_main_node(action):
            entities.extend((isolated_main_node, live_main_node))
            replaced_main_nodes += 1
        else:
            entities.append(action)

    if replaced_main_nodes != 1:
        raise RuntimeError(
            'production launch must contain exactly one main_node action'
        )

    return LaunchDescription([
        test_profile_arg,
        live_drive_arg,
        *entities,
    ])
