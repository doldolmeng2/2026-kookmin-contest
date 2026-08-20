"""Safety-preserving lane-only profile with isolated output by default.

lane_node는 /lane_segmentation_mask (PIDNet-S 출력)를 구독하므로 이 프로파일도
pidnet_inference를 함께 띄운다. 없으면 /lane_offset이 아예 나오지 않아
preflight가 FAIL 한다.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


OFFLINE_MOTOR_TOPIC = "/kmu_main_offline/xycar_motor"


def generate_launch_description():
    live_drive = LaunchConfiguration("live_drive")
    show_debug = LaunchConfiguration("show_debug")
    udp_motor_bridge = LaunchConfiguration("udp_motor_bridge")

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
        DeclareLaunchArgument(
            "udp_motor_bridge", default_value="true", choices=("false", "true"),
            description="Forward live lane-only motor output to the ROS1 UDP receiver",
        ),
        DeclareLaunchArgument(
            "pidnet_model",
            default_value=os.path.join(
                get_package_share_directory("segmentation_tools"),
                "model", "pidnet_s_best.pt",
            ),
            description="PIDNet-S checkpoint path",
        ),
        DeclareLaunchArgument(
            "pidnet_lane_classes",
            default_value="[1]",
            description=(
                "/lane_segmentation_mask 로 내보낼 PIDNet 클래스 "
                "(1=center_lane, 2=left_solid, 3=right_solid). lane_node는 선을 "
                "하나만 피팅하므로 기본값은 중앙선 단독인 [1] 이다."
            ),
        ),
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
    live_motor_bridge = Node(
        package="main", executable="udp_motor_bridge", name="udp_motor_bridge",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", live_drive, "' == 'true' and '", udp_motor_bridge,
            "' == 'true'",
        ])),
    )
    resize_node = Node(
        package="image_resize",
        executable="resize_node",
        name="resize_node",
        output="screen",
    )
    pidnet_node = Node(
        package="segmentation_tools",
        executable="pidnet_inference",
        name="pidnet_inference_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("pidnet_model"),
            "input_topic": "/resized_image",
            "mask_topic": "/lane_segmentation_mask",
            "class_topic": "/pidnet_class_map",
            "lane_classes": ParameterValue(
                LaunchConfiguration("pidnet_lane_classes"), value_type=List[int]
            ),
            "device": "auto",
        }],
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
        live_motor_bridge,
        resize_node,
        pidnet_node,
        lane_node,
        preflight,
        live_preflight,
        camera,
        lidar,
    ])
