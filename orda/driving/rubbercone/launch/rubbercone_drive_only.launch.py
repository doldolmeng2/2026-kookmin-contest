from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui = LaunchConfiguration("enable_gui")
    enable_far_curve_hint = LaunchConfiguration("enable_far_curve_hint")
    drive_speed = LaunchConfiguration("drive_speed")
    curve_speed = LaunchConfiguration("curve_speed")
    recovery_speed = LaunchConfiguration("recovery_speed")
    target_lookahead = LaunchConfiguration("target_lookahead")
    offset_gain = LaunchConfiguration("offset_gain")
    max_angle = LaunchConfiguration("max_angle")

    return LaunchDescription([
        DeclareLaunchArgument("enable_gui", default_value="true"),
        DeclareLaunchArgument("enable_far_curve_hint", default_value="false"),
        DeclareLaunchArgument("drive_speed", default_value="12.0"),
        DeclareLaunchArgument("curve_speed", default_value="8.0"),
        DeclareLaunchArgument("recovery_speed", default_value="5.5"),
        DeclareLaunchArgument("target_lookahead", default_value="0.55"),
        DeclareLaunchArgument("offset_gain", default_value="220.0"),
        DeclareLaunchArgument("max_angle", default_value="45.0"),

        Node(
            package="rubbercone",
            executable="rubbercone_node",
            name="rubbercone_node",
            output="screen",
            parameters=[{
                "enable_gui": enable_gui,
                "enable_far_curve_hint": enable_far_curve_hint,
                "scan_max_range": 1.10,
                "max_lateral_distance": 0.85,
                "boundary_points": 4,
                "max_boundary_curvature": 2.50,
                "far_scan_max_range": 1.80,
                "min_far_curve_cones": 3,
                "far_curve_min_x_span": 0.35,
                "target_lookahead": target_lookahead,
                "curve_target_lookahead": 0.45,
                "offset_gain": offset_gain,
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
                "max_angle": max_angle,
                "min_drive_confidence": 20,
                "full_speed_confidence": 85,
                "max_cone_age_s": 0.30,
                "publish_session_active": True,
            }],
        ),
    ])
