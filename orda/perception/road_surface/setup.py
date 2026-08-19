from setuptools import setup


package_name = "road_surface"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/road_surface.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orda",
    maintainer_email="hyerica.orda@gmail.com",
    description="PIDNet class-map road-surface semantic producer",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "road_surface_node = road_surface.road_surface_node:main",
        ],
    },
)
