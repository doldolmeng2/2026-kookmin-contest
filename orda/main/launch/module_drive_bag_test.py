# ─────────────────────────────────────────────────────────────────────────────
# module_drive_bag_test.py
#
# 역할: ROS bag 파일 재생 테스트용 런치 파일
#
# module_drive.py와의 차이점:
#   - 하드웨어 드라이버 (카메라, LiDAR) 런치 파일 미포함
#     → bag 파일이 /image_raw, /scan 등 토픽을 직접 재생하기 때문
#   - joy_node 미포함 (하드웨어 없음)
#   - main_node의 모터 명령을 /kmu_main_offline/xycar_motor로 격리
#
# 시작되는 노드:
#   main_node, rubbercone_node,
#   lane_node, object_yolo_node, object_node
#
# 신호등 인식은 traffic_light 패키지(traffic_node)가 아니라 object_detection
# 패키지(object_yolo_node.py + object_node)가 직접 한다. traffic_node를 같이
# 띄우면 /traffic_boxes 퍼블리셔가 겹친다 — 절대 같이 띄우지 말 것.
# ─────────────────────────────────────────────────────────────────────────────

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    object_detection_config = os.path.join(
        get_package_share_directory('object_detection'),
        'config',
        'object_detection.yaml',
    )

    # ── 런치 인수: main_node 초기 모드 / mission test entry ────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='1',
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
        default_value='2',
        description=(
            '격리된 bag-test 시작 번호 '
            '(0=race, 1=wait_green, 2=lane_center, 3=lane_1, '
            '4=lane_2, 5=cone, 6=fixed, 7=overtake, 8=shortcut)'
        )
    )
    test_profile = LaunchConfiguration('test_profile')
    live_drive_arg = DeclareLaunchArgument(
        'live_drive', default_value='false', choices=('false', 'true'),
        description='Publish to the physical motor topic only when explicitly enabled',
    )
    live_drive = LaunchConfiguration('live_drive')
    udp_motor_bridge_arg = DeclareLaunchArgument(
        'udp_motor_bridge', default_value='false', choices=('false', 'true'),
        description='Forward live motor output to the ROS1 UDP receiver only when enabled',
    )
    udp_motor_bridge = LaunchConfiguration('udp_motor_bridge')
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='상태 OpenCV 창 표시 여부'
    )
    show_debug = LaunchConfiguration('show_debug')
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
            '기본값은 중앙선 단독인 [1] 이다.'
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
        default_value='4',
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
        default_value='0.85',
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
        default_value='4',
        description='한쪽 경계 곡선 피팅에 사용할 콘 개수'
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
        default_value='0.55',
        description='라바콘 경로 목표점 전방 거리 (m)'
    )
    rubbercone_target_lookahead = LaunchConfiguration('rubbercone_target_lookahead')
    rubbercone_curve_target_lookahead_arg = DeclareLaunchArgument(
        'rubbercone_curve_target_lookahead',
        default_value='0.40',
        description='커브 판단 시 라바콘 목표점 전방 거리 (m)'
    )
    rubbercone_curve_target_lookahead = LaunchConfiguration(
        'rubbercone_curve_target_lookahead'
    )
    rubbercone_nominal_half_width_arg = DeclareLaunchArgument(
        'rubbercone_nominal_half_width',
        default_value='0.30',
        description='한쪽 경계만 보일 때 사용하는 초기 통로 반폭 (m)'
    )
    rubbercone_nominal_half_width = LaunchConfiguration('rubbercone_nominal_half_width')
    rubbercone_offset_gain_arg = DeclareLaunchArgument(
        'rubbercone_offset_gain',
        default_value='220.0',
        description='라바콘 목표점(m)에서 조향 오프셋으로 변환하는 이득'
    )
    rubbercone_offset_gain = LaunchConfiguration('rubbercone_offset_gain')
    rubbercone_offset_limit_arg = DeclareLaunchArgument(
        'rubbercone_offset_limit',
        default_value='45.0',
        description='라바콘 조향 오프셋 안전 한계'
    )
    rubbercone_offset_limit = LaunchConfiguration('rubbercone_offset_limit')
    pidnet_show_visualization_arg = DeclareLaunchArgument(
        'pidnet_show_visualization',
        default_value='false',
        description=(
            'PIDNet 세그멘테이션 결과를 OpenCV 창으로 띄운다. 주행 스택 없이 '
            '세그멘테이션만 볼 때는 module_pidnet_bag_preview.py 가 더 가볍다. '
            '창 없이 보려면 /pidnet_overlay 를 rqt_image_view 로 열면 된다.'
        )
    )
    pidnet_show_visualization = LaunchConfiguration('pidnet_show_visualization')
    pidnet_roi_crop_arg = DeclareLaunchArgument(
        'pidnet_roi_crop_visualization',
        default_value='true',
        description=(
            '시각화 창을 라벨 ROI(하단 40%, y>=216)만 잘라 보여줄지 여부. '
            '학습 라벨이 그 위쪽을 채점하지 않으므로 전체 프레임을 보면 상단의 '
            '근거 없는 예측을 결함으로 오해하기 쉽다.'
        )
    )
    pidnet_roi_crop_visualization = LaunchConfiguration(
        'pidnet_roi_crop_visualization'
    )
    mode_publish_diagnostic_arg = DeclareLaunchArgument(
        'mode_publish_diagnostic',
        default_value='false',
        description=(
            '/mode_info 값이 바뀔 때마다 그 순간의 FSM 상태·객체 id·스레드·'
            '사이클 번호를 함께 남긴다. 발행과 FSM 상태가 어긋나는 원인을 '
            '좁힐 때만 켠다.'
        )
    )
    rubbercone_max_corridor_width_arg = DeclareLaunchArgument(
        'rubbercone_max_corridor_width',
        default_value='1.40',
        description=(
            '라바콘 진입으로 인정할 통로 폭 상한 (m). 2026-08-13 bag 실측: 실제 '
            '게이트 중앙값 0.83, 최대 1.26 / 오진입 지점 1.06 이상. 1.10 부근에서 '
            '갈리지만 근거가 bag 하나뿐이라 기본값은 종전 값을 유지한다.'
        )
    )
    rubbercone_max_corridor_width = LaunchConfiguration(
        'rubbercone_max_corridor_width'
    )
    rubbercone_min_corridor_width_arg = DeclareLaunchArgument(
        'rubbercone_min_corridor_width',
        default_value='0.35',
        description='라바콘 진입으로 인정할 통로 폭 하한 (m)'
    )
    rubbercone_min_corridor_width = LaunchConfiguration(
        'rubbercone_min_corridor_width'
    )
    rubbercone_enable_gui_arg = DeclareLaunchArgument(
        'rubbercone_enable_gui',
        default_value='false',
        description='라바콘 LiDAR 인식 디버그 창 표시 여부'
    )
    rubbercone_enable_gui = LaunchConfiguration('rubbercone_enable_gui')
    rubbercone_enabled_arg = DeclareLaunchArgument(
        'rubbercone_enabled',
        default_value='true',
        choices=('false', 'true'),
        description=(
            'Obstacle-only shadow regression에서 perception interference를 '
            '분리하기 위한 test switch'
        ),
    )
    rubbercone_enabled = LaunchConfiguration('rubbercone_enabled')
    rubbercone_enable_far_curve_hint_arg = DeclareLaunchArgument(
        'rubbercone_enable_far_curve_hint',
        default_value='false',
        description='먼 라바콘 기반 커브 힌트 사용 여부'
    )
    rubbercone_enable_far_curve_hint = LaunchConfiguration(
        'rubbercone_enable_far_curve_hint'
    )
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
        'detector_model_path', default_value=''
    )
    detector_model_path = LaunchConfiguration('detector_model_path')
    perception_camera_topic_arg = DeclareLaunchArgument(
        'perception_camera_topic', default_value='/resized_image'
    )
    perception_camera_topic = LaunchConfiguration('perception_camera_topic')

    # ── 소프트웨어 노드 (하드웨어 드라이버 제외) ────────────────────────────
    main_parameters = [{
        'mode': mode,
        'lane_target': lane_target,
        'test_profile': ParameterValue(test_profile, value_type=str),
        'show_debug': show_debug,
        'mode_publish_diagnostic': ParameterValue(
            LaunchConfiguration('mode_publish_diagnostic'), value_type=bool
        ),
        'use_sim_time': True,
    }]
    isolated_main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=main_parameters,
        remappings=[('xycar_motor', '/kmu_main_offline/xycar_motor')],
        condition=UnlessCondition(live_drive),
    )
    live_main_node = Node(
        package='main', executable='main_node', name='main_node', output='screen',
        parameters=main_parameters,
        condition=IfCondition(live_drive),
    )
    live_motor_bridge = Node(
        package='main', executable='udp_motor_bridge', name='udp_motor_bridge',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", live_drive, "' == 'true' and '", udp_motor_bridge,
            "' == 'true'",
        ])),
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
            'curve_target_lookahead': rubbercone_curve_target_lookahead,
            'nominal_half_width': rubbercone_nominal_half_width,
            'min_corridor_width': rubbercone_min_corridor_width,
            'max_corridor_width': rubbercone_max_corridor_width,
            'offset_gain': rubbercone_offset_gain,
            'offset_limit': rubbercone_offset_limit,
            'enable_far_curve_hint': rubbercone_enable_far_curve_hint,
            'far_scan_max_range': 1.80,
            'min_far_curve_cones': 3,
            'far_curve_min_x_span': 0.35,
            'max_boundary_curvature': 2.50,
            'curve_slope_threshold': 0.22,
            'curve_curvature_threshold': 0.65,
            'slope_feedforward_gain': 8.0,
            'curvature_feedforward_gain': 3.0,
            'one_side_slope_feedforward_gain': 10.0,
            'one_side_curvature_feedforward_gain': 3.5,
            'max_offset_step': 20.0,
            'curve_max_offset_step': 26.0,
            'enable_gui': rubbercone_enable_gui,
        }],
        condition=IfCondition(rubbercone_enabled),
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
            'show_visualization': ParameterValue(
                pidnet_show_visualization, value_type=bool
            ),
            'roi_crop_visualization': ParameterValue(
                pidnet_roi_crop_visualization, value_type=bool
            ),
        }],
    )
    road_surface_node = Node(
        package='road_surface',
        executable='road_surface_node',
        name='road_surface_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
        parameters=[{
            # pidnet 과 같은 클래스 목록을 받아야 의미가 어긋나지 않는다.
            'center_classes': pidnet_lane_classes,
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
            'use_sim_time': True,
        }],
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
    preflight_node = Node(
        package='main',
        executable='kmu_preflight',
        name='bag_preflight',
        output='screen',
        parameters=[{
            'required_topics': ['/lane_offset', '/scan', '/object_info'],
            'motor_output_topic': '/kmu_main_offline/xycar_motor',
            'require_motor_subscriber': False,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        mode_arg,
        lane_target_arg,
        test_profile_arg,
        live_drive_arg,
        udp_motor_bridge_arg,
        show_debug_arg,
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
        rubbercone_curve_target_lookahead_arg,
        rubbercone_nominal_half_width_arg,
        rubbercone_min_corridor_width_arg,
        rubbercone_max_corridor_width_arg,
        mode_publish_diagnostic_arg,
        pidnet_show_visualization_arg,
        pidnet_roi_crop_arg,
        rubbercone_offset_gain_arg,
        rubbercone_offset_limit_arg,
        rubbercone_enable_gui_arg,
        rubbercone_enabled_arg,
        rubbercone_enable_far_curve_hint_arg,
        object_enable_gui_arg,
        detector_model_path_arg,
        perception_camera_topic_arg,
        isolated_main_node,
        live_main_node,
        live_motor_bridge,
        rubbercone_node,
        pidnet_node,
        road_surface_node,
        lane_node,
        object_yolo_node,
        object_node,
        preflight_node,
    ])
