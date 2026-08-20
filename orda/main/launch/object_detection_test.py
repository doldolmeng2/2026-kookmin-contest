# ─────────────────────────────────────────────────────────────────────────────
# object_detection_test.py
#
# 역할: object_detection 패키지(object_yolo_node.py + object_node)를 단독으로
#       검증하기 위한 최소 런치 파일. main_node 등 나머지 스택은 띄우지 않는다.
#       lane_node 는 포함한다 — /lane_fit 이 있어야 object_node 의 차선(1/2차선)
#       판정이 동작한다(없으면 /object_info 의 차량 위치 필드가 항상 0으로 나온다).
#
# 시작되는 노드: resize_node, lane_node, object_yolo_node, object_node
#
# 사용 예:
#   ros2 launch main object_detection_test.py
#   (다른 터미널) ros2 bag play <bag> --clock --topics /image_raw /scan
#
# 이 머신에서 별개 `ros2 run` 프로세스를 터미널마다 따로 띄우면 ROS2 디스커버리가
# 안 잡히는 경우가 있었다(ros2 topic list 가 빈 채로 멈춤). 하나의 `ros2 launch`
# 아래 자식 프로세스로 띄우면 안정적으로 통신됐으므로 이 파일로 묶는다.
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='CAMERA VIEW / OBJECT DEBUG 디버그 창 표시 여부'
    )
    enable_gui = LaunchConfiguration('enable_gui')

    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen',
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
    )
    object_yolo_node = Node(
        package='object_detection',
        executable='object_yolo_node.py',
        name='object_yolo_node',
        output='screen',
    )
    object_node = Node(
        package='object_detection',
        executable='object_node',
        name='object_node',
        output='screen',
        parameters=[{'enable_gui': enable_gui}],
    )

    return LaunchDescription([
        enable_gui_arg,
        resize_node,
        lane_node,
        object_yolo_node,
        object_node,
    ])
