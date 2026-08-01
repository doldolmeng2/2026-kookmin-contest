# ─────────────────────────────────────────────────────────────────────────────
# manual_drive.launch.py
#
# 역할: Xbox 컨트롤러 수동 주행 런치 파일
#
# 시작되는 노드:
#   joy_node    - ROS joy 패키지의 조이스틱 드라이버 (/joy 발행)
#   joystic     - joy → xycar_motor 변환 노드 (manual_drive 패키지)
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 조이스틱 드라이버: /dev/input/js0 → /joy 발행
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # 수동 주행 변환 노드: /joy → xycar_motor 변환
    joystic_node = Node(
        package='manual_drive',
        executable='joystic',  # setup.py console_scripts에 등록된 이름
        name='joystic_node',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        joystic_node,
    ])
