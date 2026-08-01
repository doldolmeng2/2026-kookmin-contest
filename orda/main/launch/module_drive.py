# ─────────────────────────────────────────────────────────────────────────────
# module_drive.py
#
# 역할: 자율주행 전체 시스템 런치 파일 (실제 Xycar 하드웨어 구동용)
#
# 시작되는 노드 목록:
#   main_node       - 상태 머신 및 모터 제어 (main 패키지)
#   traffic_node    - 신호등 검출 (traffic_light 패키지)
#   rubbercone_node - 라바콘 구간 LiDAR 오프셋 (rubbercone 패키지)
#   resize_node     - 카메라 영상 640×360 리사이즈 (image_resize 패키지)
#   lane_node       - 차선 검출 및 오프셋 발행 (lane_detection 패키지)
#   object_node     - 장애물 검출 (object_detection 패키지)
#   joy_node        - Xbox 컨트롤러 입력 (joy 패키지)
#
# 포함되는 런치 파일:
#   xycar_cam.launch.py       - 카메라 드라이버
#   xycar_lidar.launch.py     - LiDAR 드라이버
#   xycar_ultrasonic.launch.py- 초음파 드라이버
#
# 런치 인수:
#   mode (기본값: 0) - main_node 초기 주행 모드
#   show_debug (기본값: false) - 제어 지연을 피하기 위한 상태 창 비활성화
#   rubbercone_offset_filter_alpha (기본값: 0.80) - 라바콘 오프셋 EMA 계수
#   rubbercone_end_missing_frames (기본값: 3) - 라바콘 종료 판정 누락 프레임 수
#   rubbercone_scan_max_range (기본값: 1.30m) - 라바콘 경계 탐색 최대 거리
#   rubbercone_target_lookahead (기본값: 0.70m) - 경로 목표점 전방 거리
#   rubbercone_nominal_half_width (기본값: 0.30m) - 한쪽 경계만 보일 때의 초기 반폭
#   rubbercone_offset_gain (기본값: 230) - 목표점(m)→조향 오프셋 변환 이득
#   rubbercone_offset_limit (기본값: 40) - LiDAR 오프셋 안전 한계
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── 런치 인수: main_node 초기 모드 ──────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='main_node 초기 주행 모드 (0=TRAFFIC_WAIT)'
    )
    mode = LaunchConfiguration('mode')
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='상태 OpenCV 창 표시 여부 (실차 제어 시 false 권장)'
    )
    show_debug = LaunchConfiguration('show_debug')
    rubbercone_offset_filter_alpha_arg = DeclareLaunchArgument(
        'rubbercone_offset_filter_alpha',
        default_value='0.80',
        description='라바콘 오프셋 EMA 계수 (높을수록 빠르게 반응)'
    )
    rubbercone_offset_filter_alpha = LaunchConfiguration('rubbercone_offset_filter_alpha')
    rubbercone_end_missing_frames_arg = DeclareLaunchArgument(
        'rubbercone_end_missing_frames',
        default_value='3',
        description='라바콘 종료 전 연속 누락 스캔 수'
    )
    rubbercone_end_missing_frames = LaunchConfiguration('rubbercone_end_missing_frames')
    rubbercone_scan_max_range_arg = DeclareLaunchArgument(
        'rubbercone_scan_max_range',
        default_value='1.30',
        description='라바콘 경계 탐색 최대 거리 (m)'
    )
    rubbercone_scan_max_range = LaunchConfiguration('rubbercone_scan_max_range')
    rubbercone_target_lookahead_arg = DeclareLaunchArgument(
        'rubbercone_target_lookahead',
        default_value='0.70',
        description='라바콘 경로 목표점 전방 거리 (m)'
    )
    rubbercone_target_lookahead = LaunchConfiguration('rubbercone_target_lookahead')
    rubbercone_nominal_half_width_arg = DeclareLaunchArgument(
        'rubbercone_nominal_half_width',
        default_value='0.30',
        description='한쪽 경계만 보일 때 사용하는 초기 통로 반폭 (m)'
    )
    rubbercone_nominal_half_width = LaunchConfiguration('rubbercone_nominal_half_width')
    rubbercone_offset_gain_arg = DeclareLaunchArgument(
        'rubbercone_offset_gain',
        default_value='230.0',
        description='라바콘 목표점(m)에서 조향 오프셋으로 변환하는 이득'
    )
    rubbercone_offset_gain = LaunchConfiguration('rubbercone_offset_gain')
    rubbercone_offset_limit_arg = DeclareLaunchArgument(
        'rubbercone_offset_limit',
        default_value='40.0',
        description='라바콘 조향 오프셋 안전 한계'
    )
    rubbercone_offset_limit = LaunchConfiguration('rubbercone_offset_limit')

    # ── 소프트웨어 노드 ──────────────────────────────────────────────────────
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{'mode': mode, 'show_debug': show_debug}],
    )
    traffic_node = Node(
        package='traffic_light',
        executable='traffic_node',
        name='traffic_node',
        output='screen',
    )
    rubbercone_node = Node(
        package='rubbercone',
        executable='rubbercone_node',
        name='rubbercone_node',
        output='screen',
        parameters=[{
            'offset_filter_alpha': rubbercone_offset_filter_alpha,
            'end_missing_frames': rubbercone_end_missing_frames,
            'scan_max_range': rubbercone_scan_max_range,
            'target_lookahead': rubbercone_target_lookahead,
            'nominal_half_width': rubbercone_nominal_half_width,
            'offset_gain': rubbercone_offset_gain,
            'offset_limit': rubbercone_offset_limit,
        }],
    )
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
    object_node = Node(
        package='object_detection',
        executable='object_node',
        name='object_node',
        output='screen',
    )
    # Xbox 컨트롤러: /dev/input/js0 장치, deadzone 0.05
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
        }],
    )

    # ── 하드웨어 드라이버 런치 파일 ──────────────────────────────────────────
    # 카메라: AnyLaunchDescriptionSource (xml/py 모두 지원)
    cam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_cam'),
                'launch/xycar_cam.launch.py'
            )
        )
    )
    # LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_lidar'),
                'launch/xycar_lidar.launch.py'
            )
        )
    )
    # 초음파
    ultrasonic_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_ultrasonic'),
                'launch/xycar_ultrasonic.launch.py'
            )
        )
    )

    return LaunchDescription([
        mode_arg,
        show_debug_arg,
        rubbercone_offset_filter_alpha_arg,
        rubbercone_end_missing_frames_arg,
        rubbercone_scan_max_range_arg,
        rubbercone_target_lookahead_arg,
        rubbercone_nominal_half_width_arg,
        rubbercone_offset_gain_arg,
        rubbercone_offset_limit_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_node,
        joy_node,
        cam_launch,
        lidar_launch,
        ultrasonic_launch,
    ])
