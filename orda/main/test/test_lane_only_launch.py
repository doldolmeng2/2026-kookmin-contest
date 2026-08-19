import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "module_lane_only.py"


def description():
    spec = importlib.util.spec_from_file_location("module_lane_only", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def text(substitutions):
    return "".join(item.text for item in substitutions)


def test_lane_only_source_is_installed_and_isolated_by_default():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert LAUNCH_PATH.is_file()
    assert "launch/module_lane_only.py" in setup_source

    launch = description()
    live_arg = next(
        entity
        for entity in launch.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "live_drive"
    )
    assert text(live_arg.default_value) == "false"

    mains = [
        entity
        for entity in launch.entities
        if isinstance(entity, Node)
        and entity.node_package == "main"
        and entity.node_executable == "main_node"
    ]
    assert len(mains) == 2
    isolated = next(
        node for node in mains if type(node.condition) is UnlessCondition
    )
    assert [
        (text(source), text(target))
        for source, target in isolated._Node__remappings
    ] == [("xycar_motor", "/kmu_main_offline/xycar_motor")]


def test_lane_only_live_profile_supplies_camera_and_scan_drivers():
    includes = [
        entity
        for entity in description().entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    assert all(type(include.condition) is IfCondition for include in includes)


def test_lane_only_has_one_live_only_udp_bridge():
    bridges = [
        entity for entity in description().entities
        if isinstance(entity, Node)
        and entity.node_package == "main"
        and entity.node_executable == "udp_motor_bridge"
    ]
    assert len(bridges) == 1
    assert type(bridges[0].condition) is IfCondition


def test_lane_only_keeps_production_main_safety_and_one_line_preflight():
    nodes = [
        entity for entity in description().entities if isinstance(entity, Node)
    ]
    assert sum(
        node.node_package == "main" and node.node_executable == "kmu_preflight"
        for node in nodes
    ) == 2
    assert "SafetyMonitor" not in LAUNCH_PATH.read_text(encoding="utf-8")
