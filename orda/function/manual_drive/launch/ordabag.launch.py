# ─────────────────────────────────────────────────────────────────────────────
# ordabag.launch.py
#
# 역할: Xycar 하드웨어 드라이버 일괄 시작 런치 파일
#       bag 녹화 또는 센서 단독 구동 시 사용한다.
#
# 시작되는 노드 및 런치 파일:
#   xycar_cam.launch.py       - 카메라 드라이버 (/image_raw 발행)
#   xycar_ultrasonic.launch.py- 초음파 드라이버 (/xycar_ultrasonic 발행)
#   resize_node               - 640×360 리사이즈 (/resized_image 발행)
#   xycar_lidar.launch.py     - LiDAR 드라이버 (/scan 발행)
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 카메라 드라이버: /image_raw 발행
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_cam'),
                'launch', 'xycar_cam.launch.py'
            )
        )
    )

    # 초음파 드라이버: /xycar_ultrasonic 발행
    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_ultrasonic'),
                'launch', 'xycar_ultrasonic.launch.py'
            )
        )
    )

    # 카메라 영상 640×360 리사이즈 노드: /resized_image 발행
    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen',
    )

    # LiDAR 드라이버: /scan 발행
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_lidar'),
                'launch', 'xycar_lidar.launch.py'
            )
        )
    )

    return LaunchDescription([
        cam_launch,
        ultrasonic_launch,
        resize_node,
        lidar_launch,
    ])
