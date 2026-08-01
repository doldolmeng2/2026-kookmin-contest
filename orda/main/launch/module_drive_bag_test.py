# ─────────────────────────────────────────────────────────────────────────────
# module_drive_bag_test.py
#
# 역할: ROS bag 파일 재생 테스트용 런치 파일
#
# module_drive.py와의 차이점:
#   - 하드웨어 드라이버 (카메라, LiDAR) 런치 파일 미포함
#     → bag 파일이 /image_raw, /scan 등 토픽을 직접 재생하기 때문
#   - joy_node, xycar_ultrasonic 미포함 (하드웨어 없음)
#
# 시작되는 노드:
#   main_node, traffic_node, rubbercone_node,
#   resize_node, lane_node, object_node
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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

    # ── 소프트웨어 노드 (하드웨어 드라이버 제외) ────────────────────────────
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
    ])
