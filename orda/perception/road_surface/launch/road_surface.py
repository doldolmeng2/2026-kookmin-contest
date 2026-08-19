"""Launch the road-surface producer with explicitly supplied thresholds."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("class_map_topic", default_value="/pidnet_class_map"),
        DeclareLaunchArgument("output_topic", default_value="/road_surface"),
        DeclareLaunchArgument("road_min_ratio"),
        DeclareLaunchArgument("shortcut_min_ratio"),
        DeclareLaunchArgument("road_min_component_px"),
        DeclareLaunchArgument("shortcut_min_component_px"),
        DeclareLaunchArgument("roi_top", default_value="0.0"),
        DeclareLaunchArgument("roi_bottom", default_value="1.0"),
        DeclareLaunchArgument("roi_left", default_value="0.0"),
        DeclareLaunchArgument("roi_right", default_value="1.0"),
    ]
    node = Node(
        package="road_surface",
        executable="road_surface_node",
        name="road_surface_node",
        output="screen",
        parameters=[{
            name: LaunchConfiguration(name)
            for name in (
                "class_map_topic",
                "output_topic",
                "road_min_ratio",
                "shortcut_min_ratio",
                "road_min_component_px",
                "shortcut_min_component_px",
                "roi_top",
                "roi_bottom",
                "roi_left",
                "roi_right",
            )
        }],
    )
    return LaunchDescription([*arguments, node])
