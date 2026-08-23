# ─────────────────────────────────────────────────────────────────────────────
# module_pidnet_bag_preview.py
#
# 역할: rosbag2를 재생하면서 PIDNet-S 세그멘테이션 결과만 시각화 창으로 확인하는
#       가벼운 테스트 런치 파일. 주행 스택(main/traffic/rubbercone/lane/object) 없이
#       pidnet_inference_node 하나만 띄운다.
#
# module_drive_bag_test.py와의 차이점:
#   - resize_node를 띄우지 않는다. 재생 대상 bag(예: roabag2_gill_drive_test)은
#     실주행 중 이미 만들어진 /resized_image를 그대로 담고 있으므로, resize_node를
#     같이 띄우면 같은 토픽에 두 발행자가 겹쳐써서 프레임이 섞인다.
#   - lane/object/traffic/rubbercone 등 주행 로직 노드를 띄우지 않는다.
#   - roi_crop_visualization 파라미터로 시각화 창을 라벨링 ROI(하단 40%, y>=216)만
#     잘라서 보여준다 — 학습 라벨이 애초에 그 위쪽을 채점하지 않으므로, 전체 프레임을
#     보면서 상단의 근거 없는 예측을 "잘 안 된다"고 착각하지 않기 위함.
#
# 사용법:
#   ros2 launch /home/gill/xycar_yet/src/orda/main/launch/module_pidnet_bag_preview.py
#   (재빌드해서 `ros2 launch main module_pidnet_bag_preview.py`로 쓰려면
#    src/orda/main/setup.py의 launch 파일 목록에 이 파일이 이미 등록돼 있음)
#
# 파라미터:
#   bag_path: 재생할 rosbag2 디렉토리 (기본: roabag2_gill_drive_test)
#   bag_rate: 재생 속도 배율 (기본 1.0; 천천히 보려면 0.3~0.5 추천)
#   pidnet_model: 체크포인트 경로 (기본: 가장 성능이 좋았던 v3_noweight/best.pt)
#   roi_crop_visualization: true면 시각화 창을 라벨 ROI만 잘라서 보여줌 (기본 true)
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
        description='재생할 rosbag2 디렉토리 경로'
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

    roi_crop_arg = DeclareLaunchArgument(
        'roi_crop_visualization',
        default_value='true',
        description='시각화 창을 라벨 ROI(하단 40%)만 잘라서 보여줄지 여부'
    )
    roi_crop_visualization = LaunchConfiguration('roi_crop_visualization')

    pidnet_node = Node(
        package='segmentation_tools',
        executable='pidnet_inference',
        name='pidnet_inference_node',
        output='screen',
        parameters=[{
            'model_path': pidnet_model,
            'input_topic': '/resized_image',
            'class_topic': '/pidnet_class_map',
            'device': 'auto',
            'show_visualization': True,
            'roi_crop_visualization': roi_crop_visualization,
        }],
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--rate', bag_rate],
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        bag_rate_arg,
        pidnet_model_arg,
        roi_crop_arg,
        pidnet_node,
        bag_play,
    ])
