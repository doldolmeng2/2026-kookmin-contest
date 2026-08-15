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
#   rubbercone_scan_max_range (기본값: 1.10m) - 라바콘 경계 탐색 최대 거리
#   rubbercone_scan_max_angle (기본값: 85deg) - 탐색 좌우 최대 각도
#   rubbercone_max_lateral_distance (기본값: 0.70m) - 탐색 좌우 최대 편차
#   rubbercone_max_cone_centers (기본값: 6) - 사용할 콘 개수 (가까운 순)
#   rubbercone_boundary_points (기본값: 3) - 경계 직선 피팅에 쓸 콘 개수
#   rubbercone_target_lookahead (기본값: 0.70m) - 경로 목표점 전방 거리
#   rubbercone_nominal_half_width (기본값: 0.30m) - 한쪽 경계만 보일 때의 초기 반폭
#   rubbercone_offset_gain (기본값: 150) - 목표점(m)→조향 오프셋 변환 이득
#   rubbercone_offset_limit (기본값: 45) - LiDAR 오프셋 안전 한계
#   rubbercone_enable_gui (기본값: true) - 라바콘 LiDAR 인식 디버그 창 표시
#   object_enable_gui (기본값: true) - 장애물 검출 디버그 창 표시
#     (성능에 영향. 기록 주행 시 object_enable_gui:=false 권장)
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
        description=(
            '초기 모드 번호: 0=INIT, 1=WAIT_TRAFFIC, 2=LANE, 3=CONE, '
            '4=FIXED, 5=OVERTAKE, 6=SHORTCUT, 7=FINISH, 8=STOP'
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
        default_value='1.10',
        description='라바콘 경계 탐색 최대 거리 (m)'
    )
    rubbercone_scan_max_range = LaunchConfiguration('rubbercone_scan_max_range')
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
        description='라바콘 LiDAR 인식 디버그 창 표시 여부 (기록 주행 시 false 권장)'
    )
    rubbercone_enable_gui = LaunchConfiguration('rubbercone_enable_gui')
    object_enable_gui_arg = DeclareLaunchArgument(
        'object_enable_gui',
        default_value='false',
        description=(
            '장애물 검출 디버그 창(CAMERA VIEW / OBJECT DEBUG) 표시 여부. '
            '기본 false. 켜두면 영상 표시가 CPU를 사용해 '
            '/object_info 와 /lane_offset 이 느려지므로(실측: 카메라 18.8 Hz '
            '입력에 인지 5.9 Hz 출력), 기록 주행·성능 측정 시에는 '
            'object_enable_gui:=false 로 끌 것.'
        )
    )
    object_enable_gui = LaunchConfiguration('object_enable_gui')

    # ── 소프트웨어 노드 ──────────────────────────────────────────────────────
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{
            'mode': mode,
            'lane_target': lane_target,
            'show_debug': show_debug,
        }],
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
        parameters=[{'enable_gui': object_enable_gui}],
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
        lane_target_arg,
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
        joy_node,
        cam_launch,
        lidar_launch,
        ultrasonic_launch,
    ])
