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

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='CAMERA VIEW / OBJECT DEBUG 디버그 창 표시 여부'
    )
    enable_gui = LaunchConfiguration('enable_gui')
    pidnet_model_arg = DeclareLaunchArgument(
        'pidnet_model',
        default_value=os.path.join(
            get_package_share_directory('segmentation_tools'),
            'model', 'pidnet_s_best.pt',
        ),
        description='PIDNet-S checkpoint path'
    )
    pidnet_lane_classes_arg = DeclareLaunchArgument(
        'pidnet_lane_classes',
        default_value='[1]',
        description='/lane_segmentation_mask 로 내보낼 PIDNet 클래스 (1=center_lane)'
    )

    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen',
    )
    # lane_node가 /lane_segmentation_mask 를 구독하므로 여기서도 PIDNet이
    # 필요하다. 없으면 /lane_fit 이 나오지 않아 object_node 의 1/2차선 판정이
    # 항상 0으로 나온다 (이 파일이 lane_node 를 포함하는 이유가 그것이다).
    pidnet_node = Node(
        package='segmentation_tools',
        executable='pidnet_inference',
        name='pidnet_inference_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('pidnet_model'),
            'input_topic': '/resized_image',
            'mask_topic': '/lane_segmentation_mask',
            'class_topic': '/pidnet_class_map',
            'lane_classes': ParameterValue(
                LaunchConfiguration('pidnet_lane_classes'), value_type=List[int]
            ),
            'device': 'auto',
        }],
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=[{
            # pidnet 과 같은 클래스 목록을 받아야 의미가 어긋나지 않는다.
            'center_classes': ParameterValue(
                LaunchConfiguration('pidnet_lane_classes'), value_type=List[int]
            ),
        }],
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
        pidnet_model_arg,
        pidnet_lane_classes_arg,
        resize_node,
        pidnet_node,
        lane_node,
        object_yolo_node,
        object_node,
    ])
