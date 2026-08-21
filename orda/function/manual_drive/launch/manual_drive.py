"""Safe-by-default Xbox manual-drive launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    live_drive = LaunchConfiguration('live_drive')
    manual_parameters = {
        'steering_axis': LaunchConfiguration('steering_axis'),
        'steering_scale': LaunchConfiguration('steering_scale'),
        'speed_axis': LaunchConfiguration('speed_axis'),
        'speed_scale': LaunchConfiguration('speed_scale'),
        'deadman_button': LaunchConfiguration('deadman_button'),
        'max_abs_steering': LaunchConfiguration('max_abs_steering'),
        'max_abs_speed': LaunchConfiguration('max_abs_speed'),
        'joy_timeout_s': LaunchConfiguration('joy_timeout_s'),
        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
    }

    arguments = [
        DeclareLaunchArgument(
            'live_drive',
            default_value='false',
            choices=['false', 'true'],
            description=(
                'false isolates output on /manual_test/xycar_motor; '
                'true publishes to the real /xycar_motor topic'
            ),
        ),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.05'),
        DeclareLaunchArgument('joy_autorepeat_rate', default_value='20.0'),
        DeclareLaunchArgument('steering_axis', default_value='0'),
        DeclareLaunchArgument('steering_scale', default_value='-100.0'),
        DeclareLaunchArgument('speed_axis', default_value='4'),
        DeclareLaunchArgument('speed_scale', default_value='50.0'),
        DeclareLaunchArgument(
            'deadman_button',
            default_value='-1',
            description=(
                'Xbox button index held for motion; -1 is safe STOP-only until '
                'the controller mapping is verified'
            ),
        ),
        DeclareLaunchArgument('max_abs_steering', default_value='45.0'),
        DeclareLaunchArgument('max_abs_speed', default_value='7.5'),
        DeclareLaunchArgument('joy_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
    ]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_name': LaunchConfiguration('joy_dev'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }],
    )

    isolated_manual_node = Node(
        package='manual_drive',
        executable='joystic',
        name='manual_drive_node',
        output='screen',
        parameters=[manual_parameters],
        remappings=[('xycar_motor', '/manual_test/xycar_motor')],
        condition=UnlessCondition(live_drive),
    )
    live_manual_node = Node(
        package='manual_drive',
        executable='joystic',
        name='manual_drive_node',
        output='screen',
        parameters=[manual_parameters],
        condition=IfCondition(live_drive),
    )

    return LaunchDescription([
        *arguments,
        LogInfo(
            condition=UnlessCondition(live_drive),
            msg=(
                'MANUAL TEST ISOLATED: motor output is remapped to '
                '/manual_test/xycar_motor'
            ),
        ),
        LogInfo(
            condition=IfCondition(live_drive),
            msg=(
                'DANGER: LIVE MANUAL DRIVE ENABLED. Do not run module_drive.py '
                'or module_drive_mission_test.py live_drive:=true concurrently.'
            ),
        ),
        joy_node,
        isolated_manual_node,
        live_manual_node,
    ])
