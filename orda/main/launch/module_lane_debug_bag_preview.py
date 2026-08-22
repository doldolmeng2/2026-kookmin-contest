# ─────────────────────────────────────────────────────────────────────────────
# module_lane_debug_bag_preview.py
#
# 역할: rosbag2를 재생하면서 lane_node의 통합 모니터 창(Lane Drive Monitor:
#       CAMERA 라벨 ROI 오버레이 + VEHICLE)으로 차선주행 인지를 확인하는
#       가벼운 테스트 런치 파일. 주행 스택(main/traffic/rubbercone/object) 없이
#       pidnet_inference_node + lane_node 둘만 띄운다.
#
# module_pidnet_bag_preview.py와의 차이점:
#   - PIDNet 자체 시각화 창 대신 lane_node의 통합 모니터 창(카메라 위에
#     중앙선 세그멘테이션을 겹쳐 보여주고, VEHICLE 패널로 조향/속도도 함께
#     확인)을 띄운다.
#   - resize_node는 띄우지 않는다. 재생 대상 bag이 실주행 중 이미 만들어진
#     /resized_image를 그대로 담고 있으므로, resize_node를 같이 띄우면 같은
#     토픽에 두 발행자가 겹쳐써서 프레임이 섞인다.
#
# 사용법:
#   ros2 launch main module_lane_debug_bag_preview.py bag_path:=/path/to/bag
#
# 파라미터:
#   bag_path: 재생할 rosbag2 디렉토리
#   bag_rate: 재생 속도 배율 (기본 1.0; 천천히 보려면 0.3~0.5 추천)
#   pidnet_model: 체크포인트 경로 (기본: 가장 성능이 좋았던 v3_noweight/best.pt)
#   lane_debug_detail: true면 보조 차선 창(ROI Polygon / BEV-PIDNet-Center-Lane /
#     Lane View + Offset)까지 추가로 띄운다 (기본 false)
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/gill/xycar_yet/bags/roabag2_gill_drive_test',
        description='재생할 rosbag2 디렉터리 경로'
    )
    bag_path = LaunchConfiguration('bag_path')

    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='1.0',
        description='ros2 bag play 재생 속도 배율'
    )
    bag_rate = LaunchConfiguration('bag_rate')

    pidnet_model_arg = DeclareLaunchArgument(
        'pidnet_model',
        default_value=os.path.join(
            get_package_share_directory('segmentation_tools'),
            'model', 'pidnet_s_best.pt',
        ),
        description='PIDNet-S checkpoint path'
    )
    pidnet_model = LaunchConfiguration('pidnet_model')

    lane_debug_detail_arg = DeclareLaunchArgument(
        'lane_debug_detail',
        default_value='false',
        choices=('false', 'true'),
        description=(
            '보조 차선 창까지 표시 (ROI Polygon / BEV-PIDNet-Center-Lane / '
            'Lane View + Offset)'
        ),
    )
    lane_debug_detail = LaunchConfiguration('lane_debug_detail')

    pidnet_node = Node(
        package='segmentation_tools',
        executable='pidnet_inference',
        name='pidnet_inference_node',
        output='screen',
        parameters=[{
            'model_path': pidnet_model,
            'input_topic': '/resized_image',
            'mask_topic': '/lane_segmentation_mask',
            'class_topic': '/pidnet_class_map',
            # lane_node는 중앙선 하나만 피팅하므로 중앙선 단독([1])만 내보낸다.
            'lane_classes': [1],
            'device': 'auto',
        }],
    )

    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=[{
            'debug_view': True,
            'debug_lane_view': lane_debug_detail,
            'camera_topic': '/resized_image',
            # 위 pidnet_node 의 lane_classes 와 같은 값이어야 한다. lane_node 가
            # /pidnet_class_map 에서 직접 중앙선을 뽑기 때문이다.
            'center_classes': [1],
        }],
        remappings=[('/mode_info', '/internal/lane_command')],
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--rate', bag_rate],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        bag_rate_arg,
        pidnet_model_arg,
        lane_debug_detail_arg,
        pidnet_node,
        lane_node,
        bag_play,
    ])
