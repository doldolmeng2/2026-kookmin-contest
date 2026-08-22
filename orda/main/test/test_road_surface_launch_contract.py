from pathlib import Path


LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def test_production_and_bag_launch_each_have_exactly_one_surface_producer():
    for name in ("module_drive.py", "module_drive_bag_test.py"):
        source = (LAUNCH_DIR / name).read_text(encoding="utf-8")
        assert source.count("package='road_surface'") == 1
        assert source.count("executable='road_surface_node'") == 1


def test_bag_surface_producer_uses_sim_time():
    source = (LAUNCH_DIR / "module_drive_bag_test.py").read_text(
        encoding="utf-8"
    )
    producer = source[source.index("road_surface_node = Node(") :]
    producer = producer[: producer.index("lane_node = Node(")]
    assert "'use_sim_time': True" in producer
