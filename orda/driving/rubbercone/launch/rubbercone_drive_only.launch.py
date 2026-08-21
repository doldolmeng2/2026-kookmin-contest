from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui = LaunchConfiguration("enable_gui")
    drive_speed = LaunchConfiguration("drive_speed")
    curve_speed = LaunchConfiguration("curve_speed")
    recovery_speed = LaunchConfiguration("recovery_speed")

    return LaunchDescription([
        DeclareLaunchArgument("enable_gui", default_value="true"),
        DeclareLaunchArgument("drive_speed", default_value="11.0"),
        DeclareLaunchArgument("curve_speed", default_value="8.0"),
        DeclareLaunchArgument("recovery_speed", default_value="5.5"),

        Node(
            package="rubbercone",
            executable="rubbercone_node",
            name="rubbercone_node",
            output="screen",
            parameters=[{
                "enable_gui": enable_gui,
                "scan_max_range": 1.10,
                "far_scan_max_range": 1.80,
                "target_lookahead": 0.70,
                "curve_target_lookahead": 0.50,
                "recovery_target_lookahead": 0.42,
                "offset_gain": 150.0,
                "offset_limit": 45.0,
                "end_missing_frames": 4,
            }],
        ),
        Node(
            package="rubbercone",
            executable="rubbercone_drive_only_node.py",
            name="rubbercone_drive_only_node",
            output="screen",
            parameters=[{
                "cruise_speed": drive_speed,
                "curve_speed": curve_speed,
                "recovery_speed": recovery_speed,
                "max_angle": 45.0,
                "min_drive_confidence": 20,
                "full_speed_confidence": 85,
                "max_cone_age_s": 0.25,
                "publish_session_active": True,
            }],
        ),
    ])
