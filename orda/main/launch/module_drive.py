# ─────────────────────────────────────────────────────────────────────────────
# module_drive.py
#
# 역할: 자율주행 전체 시스템 런치 파일 (실제 Xycar 하드웨어 구동용)
#
# 시작되는 노드 목록:
#   main_node        - 상태 머신 및 모터 제어 (main 패키지)
#   rubbercone_node  - 라바콘 구간 LiDAR 오프셋 (rubbercone 패키지)
#   resize_node      - 카메라 영상 640×360 리사이즈 (image_resize 패키지)
#   lane_node        - 차선 검출 및 오프셋 발행 (lane_detection 패키지)
#   object_yolo_node - ONNX Runtime 차량/신호등 추론 (object_detection 패키지)
#   object_node      - 장애물 검출 + 신호등 상태 판정 (object_detection 패키지)
#   joy_node         - Xbox 컨트롤러 입력 (joy 패키지)
#
# 신호등 인식은 traffic_light 패키지(traffic_node)가 아니라 object_detection
# 패키지가 직접 한다. traffic_node를 같이 띄우면 /traffic_boxes 퍼블리셔가
# 겹친다 — 절대 같이 띄우지 말 것.
#
# 포함되는 런치 파일:
#   xycar_cam.launch.py       - 카메라 드라이버
#   xycar_lidar.launch.py     - LiDAR 드라이버
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
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    object_detection_config = os.path.join(
        get_package_share_directory('object_detection'),
        'config',
        'object_detection.yaml',
    )

    # ── 런치 인수: main_node 초기 모드 ──────────────────────────────────────
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
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='상태 OpenCV 창 표시 여부 (실차 제어 시 false 권장)'
    )
    show_debug = LaunchConfiguration('show_debug')
    lane_debug_arg = DeclareLaunchArgument(
        'lane_debug',
        default_value='false',
        choices=('false', 'true'),
        description=(
            '차선 인식 OpenCV 창 표시 (SlidingWindows / Mask-PIDNet-Center-Lane / '
            'Vehicle Dynamics). 영상 표시가 CPU를 쓰므로 기록 주행에서는 끌 것.'
        )
    )
    lane_debug_detail_arg = DeclareLaunchArgument(
        'lane_debug_detail',
        default_value='false',
        choices=('false', 'true'),
        description=(
            '보조 차선 창까지 표시 (ROI Polygon / BEV-PIDNet-Center-Lane / '
            'Lane View + Offset). lane_debug:=true 일 때만 의미가 있다.'
        )
    )
    pidnet_model_arg = DeclareLaunchArgument(
        'pidnet_model',
        default_value=os.path.join(
            get_package_share_directory('segmentation_tools'),
            'model', 'pidnet_s_best.pt',
        ),
        description='PIDNet-S checkpoint path'
    )
    pidnet_model = LaunchConfiguration('pidnet_model')
    pidnet_lane_classes_arg = DeclareLaunchArgument(
        'pidnet_lane_classes',
        default_value='[1]',
        description=(
            '/lane_segmentation_mask 로 내보낼 PIDNet 클래스 '
            '(1=center_lane, 2=left_solid, 3=right_solid, 4=road, 5=shortcut). '
            'lane_node는 선을 하나만 피팅하고 1/2차선 모드는 기준 x만 옮기므로 '
            '기본값은 중앙선 단독인 [1] 이다. 경계선을 섞으면 트래커가 '
            '중앙선 대신 경계선에 붙어 오프셋이 한 차선 폭만큼 어긋날 수 있다. '
            '(/pidnet_class_map 은 이 값과 무관하게 전체 라벨을 내보내므로 '
            'road_surface 노드는 영향받지 않는다.)'
        )
    )
    pidnet_lane_classes = ParameterValue(
        LaunchConfiguration('pidnet_lane_classes'), value_type=List[int]
    )
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
            '/object_info_raw 와 /lane_offset 이 느려지므로(실측: 카메라 18.8 Hz '
            '입력에 인지 5.9 Hz 출력), 기록 주행·성능 측정 시에는 '
            'object_enable_gui:=false 로 끌 것.'
        )
    )
    object_enable_gui = LaunchConfiguration('object_enable_gui')
    detector_model_path_arg = DeclareLaunchArgument(
        'detector_model_path',
        default_value='',
        description=(
            'train10_detector_best.onnx override (empty uses package share)'
        ),
    )
    detector_model_path = LaunchConfiguration('detector_model_path')
    perception_camera_topic_arg = DeclareLaunchArgument(
        'perception_camera_topic',
        default_value='/resized_image',
    )
    perception_camera_topic = LaunchConfiguration('perception_camera_topic')
    motor_output_topic_arg = DeclareLaunchArgument(
        'motor_output_topic',
        default_value='/xycar_motor',
        description='Main motor output; production default is the fixed contract',
    )
    motor_output_topic = LaunchConfiguration('motor_output_topic')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        description='Stable CP2102 LiDAR device path overriding the YAML port',
    )
    lidar_port = LaunchConfiguration('lidar_port')
    udp_motor_bridge_arg = DeclareLaunchArgument(
        'udp_motor_bridge', default_value='true', choices=('false', 'true'),
        description='Forward the selected motor output to the local ROS1 UDP receiver',
    )
    udp_motor_bridge = LaunchConfiguration('udp_motor_bridge')

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
        remappings=[('xycar_motor', motor_output_topic)],
    )
    motor_bridge_node = Node(
        package='main', executable='udp_motor_bridge', name='udp_motor_bridge',
        output='screen', remappings=[('xycar_motor', motor_output_topic)],
        condition=IfCondition(udp_motor_bridge),
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
            'lane_classes': pidnet_lane_classes,
            'device': 'auto',
        }],
    )
    road_surface_node = Node(
        package='road_surface',
        executable='road_surface_node',
        name='road_surface_node',
        output='screen',
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=[{
            'debug_view': ParameterValue(
                LaunchConfiguration('lane_debug'), value_type=bool
            ),
            'debug_lane_view': ParameterValue(
                LaunchConfiguration('lane_debug_detail'), value_type=bool
            ),
        }],
        remappings=[('/mode_info', '/internal/lane_command')],
    )
    object_yolo_node = Node(
        package='object_detection',
        executable='object_yolo_node.py',
        name='object_yolo_node',
        output='screen',
        parameters=[object_detection_config, {
            'detector_model_path': detector_model_path,
            'camera_topic': perception_camera_topic,
        }],
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
    preflight_node = Node(
        package='main',
        executable='kmu_preflight',
        name='production_preflight',
        output='screen',
        parameters=[{
            'required_topics': [
                '/lane_offset', '/scan', '/object_info',
                '/object_info_raw', '/side_clearance',
            ],
            'motor_output_topic': motor_output_topic,
            'require_motor_subscriber': True,
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
        ),
        launch_arguments={'port': lidar_port}.items(),
    )
    return LaunchDescription([
        mode_arg,
        lane_target_arg,
        show_debug_arg,
        lane_debug_arg,
        lane_debug_detail_arg,
        pidnet_model_arg,
        pidnet_lane_classes_arg,
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
        detector_model_path_arg,
        perception_camera_topic_arg,
        motor_output_topic_arg,
        lidar_port_arg,
        udp_motor_bridge_arg,
        main_node,
        motor_bridge_node,
        rubbercone_node,
        resize_node,
        pidnet_node,
        road_surface_node,
        lane_node,
        object_yolo_node,
        object_node,
        joy_node,
        preflight_node,
        cam_launch,
        lidar_launch,
    ])
