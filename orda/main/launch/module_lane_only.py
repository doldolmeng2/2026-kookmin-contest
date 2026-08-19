"""Safety-preserving lane-only profile with isolated output by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OFFLINE_MOTOR_TOPIC = "/kmu_main_offline/xycar_motor"


def generate_launch_description():
    live_drive = LaunchConfiguration("live_drive")
    show_debug = LaunchConfiguration("show_debug")

    arguments = [
        DeclareLaunchArgument(
            "live_drive",
            default_value="false",
            choices=("false", "true"),
            description=(
                "false isolates Main output; true starts camera/LiDAR inputs "
                "and publishes /xycar_motor (motor driver still required)"
            ),
        ),
        DeclareLaunchArgument("show_debug", default_value="false"),
    ]
    parameters = [{"mode": 1, "show_debug": show_debug}]
    isolated_main = Node(
        package="main",
        executable="main_node",
        name="main_node",
        output="screen",
        parameters=parameters,
        remappings=[("xycar_motor", OFFLINE_MOTOR_TOPIC)],
        condition=UnlessCondition(live_drive),
    )
    live_main = Node(
        package="main",
        executable="main_node",
        name="main_node",
        output="screen",
        parameters=parameters,
        condition=IfCondition(live_drive),
    )
    resize_node = Node(
        package="image_resize",
        executable="resize_node",
        name="resize_node",
        output="screen",
    )
    lane_node = Node(
        package="lane_detection",
        executable="lane_node",
        name="lane_node",
        output="screen",
        remappings=[("/mode_info", "/internal/lane_command")],
    )
    preflight = Node(
        package="main",
        executable="kmu_preflight",
        name="lane_only_preflight",
        output="screen",
        parameters=[{
            "required_topics": ["/lane_offset", "/scan"],
            "motor_output_topic": OFFLINE_MOTOR_TOPIC,
            "require_motor_subscriber": False,
        }],
        condition=UnlessCondition(live_drive),
    )
    live_preflight = Node(
        package="main",
        executable="kmu_preflight",
        name="lane_only_live_preflight",
        output="screen",
        parameters=[{
            "required_topics": ["/lane_offset", "/scan"],
            "motor_output_topic": "/xycar_motor",
            "require_motor_subscriber": True,
        }],
        condition=IfCondition(live_drive),
    )
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("xycar_cam"),
                "launch/xycar_cam.launch.py",
            )
        ),
        condition=IfCondition(live_drive),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("xycar_lidar"),
                "launch/xycar_lidar.launch.py",
            )
        ),
        condition=IfCondition(live_drive),
    )
    return LaunchDescription([
        *arguments,
        isolated_main,
        live_main,
        resize_node,
        lane_node,
        preflight,
        live_preflight,
        camera,
        lidar,
    ])
