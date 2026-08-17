# ─────────────────────────────────────────────────────────────────────────────
# module_drive_bag_test.py
#
# 역할: ROS bag 파일 재생 테스트용 런치 파일
#
# module_drive.py와의 차이점:
#   - 하드웨어 드라이버 (카메라, LiDAR) 런치 파일 미포함
#     → bag 파일이 /image_raw, /scan 등 토픽을 직접 재생하기 때문
#   - joy_node, xycar_ultrasonic 미포함 (하드웨어 없음)
#   - main_node의 모터 명령을 /bag_test/xycar_motor로 격리
#
# 시작되는 노드:
#   main_node, traffic_node, rubbercone_node,
#   resize_node, lane_node, object_node
# ─────────────────────────────────────────────────────────────────────────────

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    object_detection_config = os.path.join(
        get_package_share_directory('object_detection'),
        'config',
        'object_detection.yaml',
    )

    # ── 런치 인수: main_node 초기 모드 / mission test entry ────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description=(
            '초기 모드 번호: 0=WAIT_GREEN, 1=LANE, 2=CONE, '
            '3=FIXED, 4=OVERTAKE, 5=SHORTCUT'
        )
    )
    mode = LaunchConfiguration('mode')
    lane_target_arg = DeclareLaunchArgument(
        'lane_target',
        default_value='0',
        choices=('0', '1', '2'),
        description='초기 차선 번호 (0=중앙, 1=1차선, 2=2차선)',
    )
    lane_target = LaunchConfiguration('lane_target')
    test_profile_arg = DeclareLaunchArgument(
        'test_profile',
        default_value='0',
        description=(
            '격리된 bag-test 시작 번호 '
            '(0=race, 1=wait_green, 2=lane_center, 3=lane_1, '
            '4=lane_2, 5=cone, 6=fixed, 7=overtake, 8=shortcut)'
        )
    )
    test_profile = LaunchConfiguration('test_profile')
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='상태 OpenCV 창 표시 여부'
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
    rubbercone_scan_max_angle_arg = DeclareLaunchArgument(
        'rubbercone_scan_max_angle',
        default_value='85.0',
        description='라바콘 탐색 좌우 최대 각도 (deg, 전방 기준)'
    )
    rubbercone_scan_max_angle = LaunchConfiguration('rubbercone_scan_max_angle')
    rubbercone_max_lateral_distance_arg = DeclareLaunchArgument(
        'rubbercone_max_lateral_distance',
        default_value='0.70',
        description='라바콘 탐색 좌우 최대 편차 (m, 벽/옆 코스 제거)'
    )
    rubbercone_max_lateral_distance = LaunchConfiguration('rubbercone_max_lateral_distance')
    rubbercone_max_cone_centers_arg = DeclareLaunchArgument(
        'rubbercone_max_cone_centers',
        default_value='6',
        description='경로 추정에 사용할 콘 개수 (가까운 순)'
    )
    rubbercone_max_cone_centers = LaunchConfiguration('rubbercone_max_cone_centers')
    rubbercone_boundary_points_arg = DeclareLaunchArgument(
        'rubbercone_boundary_points',
        default_value='3',
        description='한쪽 경계 직선 피팅에 사용할 콘 개수'
    )
    rubbercone_boundary_points = LaunchConfiguration('rubbercone_boundary_points')
    rubbercone_scan_max_range_arg = DeclareLaunchArgument(
        'rubbercone_scan_max_range',
        default_value='1.10',
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
        default_value='150.0',
        description='라바콘 목표점(m)에서 조향 오프셋으로 변환하는 이득'
    )
    rubbercone_offset_gain = LaunchConfiguration('rubbercone_offset_gain')
    rubbercone_offset_limit_arg = DeclareLaunchArgument(
        'rubbercone_offset_limit',
        default_value='45.0',
        description='라바콘 조향 오프셋 안전 한계'
    )
    rubbercone_offset_limit = LaunchConfiguration('rubbercone_offset_limit')
    rubbercone_enable_gui_arg = DeclareLaunchArgument(
        'rubbercone_enable_gui',
        default_value='false',
        description='라바콘 LiDAR 인식 디버그 창 표시 여부'
    )
    rubbercone_enable_gui = LaunchConfiguration('rubbercone_enable_gui')
    object_enable_gui_arg = DeclareLaunchArgument(
        'object_enable_gui',
        default_value='false',
        description=(
            '장애물 검출 디버그 창(CAMERA VIEW / OBJECT DEBUG) 표시 여부. '
            '기본 false. 켜두면 영상 표시가 CPU를 사용해 '
            '/object_info_raw 와 /lane_offset 이 느려지므로(실측: 카메라 18.8 Hz '
            '입력에 인지 5.9 Hz 출력), 기록 주행·성능 측정 시에는 '
            'object_enable_gui:=false 로 끌 것.'
        )
    )
    object_enable_gui = LaunchConfiguration('object_enable_gui')

    # ── 소프트웨어 노드 (하드웨어 드라이버 제외) ────────────────────────────
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{
            'mode': mode,
            'lane_target': lane_target,
            'test_profile': test_profile,
            'show_debug': show_debug,
            # bag --loop 재생 시 /clock 역행을 감지해 FSM을 초기화한다.
            # 반드시 `ros2 bag play ... --clock` 과 함께 사용할 것.
            'use_sim_time': True,
        }],
        remappings=[('xycar_motor', '/bag_test/xycar_motor')],
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
            'scan_max_angle': rubbercone_scan_max_angle,
            'max_lateral_distance': rubbercone_max_lateral_distance,
            'max_cone_centers': rubbercone_max_cone_centers,
            'boundary_points': rubbercone_boundary_points,
            'target_lookahead': rubbercone_target_lookahead,
            'nominal_half_width': rubbercone_nominal_half_width,
            'offset_gain': rubbercone_offset_gain,
            'offset_limit': rubbercone_offset_limit,
            'enable_gui': rubbercone_enable_gui,
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
        remappings=[('/mode_info', '/internal/lane_command')],
    )
    object_yolo_node = Node(
        package='object_detection',
        executable='object_yolo_node.py',
        name='object_yolo_node',
        output='screen',
        parameters=[object_detection_config, {'use_sim_time': True}],
    )
    object_node = Node(
        package='object_detection',
        executable='object_node',
        name='object_node',
        output='screen',
        parameters=[{
            'enable_gui': object_enable_gui,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        mode_arg,
        lane_target_arg,
        test_profile_arg,
        show_debug_arg,
        rubbercone_offset_filter_alpha_arg,
        rubbercone_end_missing_frames_arg,
        rubbercone_scan_max_range_arg,
        rubbercone_scan_max_angle_arg,
        rubbercone_max_lateral_distance_arg,
        rubbercone_max_cone_centers_arg,
        rubbercone_boundary_points_arg,
        rubbercone_target_lookahead_arg,
        rubbercone_nominal_half_width_arg,
        rubbercone_offset_gain_arg,
        rubbercone_offset_limit_arg,
        rubbercone_enable_gui_arg,
        object_enable_gui_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_yolo_node,
        object_node,
    ])
