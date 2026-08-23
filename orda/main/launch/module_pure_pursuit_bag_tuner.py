"""Replay a lane bag into the motor-isolated Pure Pursuit shadow tuner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    pidnet_model = LaunchConfiguration('pidnet_model')

    arguments = [
        DeclareLaunchArgument(
            'bag_path',
            description='rosbag2 directory containing /resized_image',
        ),
        DeclareLaunchArgument(
            'bag_rate',
            default_value='0.5',
            description='Replay rate; 0.3-0.5 is convenient for tuning',
        ),
        DeclareLaunchArgument(
            'pidnet_model',
            default_value=os.path.join(
                get_package_share_directory('segmentation_tools'),
                'model',
                'pidnet_s_best.pt',
            ),
            description='PIDNet-S checkpoint path',
        ),
        DeclareLaunchArgument(
            'lane_reacquire_fallback',
            default_value='true',
            choices=('false', 'true'),
            description='Match the production full-BEV reacquire setting',
        ),
    ]

    pidnet_node = Node(
        package='segmentation_tools',
        executable='pidnet_inference',
        name='pidnet_inference_node',
        output='screen',
        parameters=[{
            'model_path': pidnet_model,
            'input_topic': '/resized_image',
            'class_topic': '/pidnet_class_map',
            'overlay_topic': '/pidnet_overlay',
            'show_visualization': False,
            'device': 'auto',
        }],
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=[{
            'debug_view': False,
            'debug_lane_view': False,
            'enable_reacquire_full_bev_fallback': ParameterValue(
                LaunchConfiguration('lane_reacquire_fallback'),
                value_type=bool,
            ),
            # lane_node 가 /pidnet_class_map 에서 직접 중앙선을 뽑는다.
            'center_classes': [1],
        }],
        remappings=[('/mode_info', '/internal/lane_command')],
    )
    tuner_node = Node(
        package='main',
        executable='pure_pursuit_tuner',
        name='pure_pursuit_tuner',
        output='screen',
        parameters=[{
            'overlay_topic': '/pidnet_overlay',
            'offset_topic': '/lane_offset',
            'fit_topic': '/lane_fit',
            'valid_topic': '/lane_valid',
            'motor_topic': '/xycar_motor',
        }],
    )
    bag_play = ExecuteProcess(
        # Replay only upstream evidence plus the recorded reference command.
        # Replaying recorded lane outputs while lane_node republishes them
        # would mix old and newly computed /lane_offset values.
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--rate', bag_rate,
            '--topics',
            '/resized_image',
            '/mode_info',
            '/internal/lane_command',
            '/xycar_motor',
        ],
        output='screen',
    )
    delayed_bag_play = TimerAction(period=5.0, actions=[bag_play])

    return LaunchDescription([
        *arguments,
        pidnet_node,
        lane_node,
        tuner_node,
        delayed_bag_play,
    ])
