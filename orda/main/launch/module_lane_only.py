"""Safety-preserving lane-only profile with isolated output by default.

lane_node는 /pidnet_class_map (PIDNet-S 출력)을 구독하므로 이 프로파일도
pidnet_inference를 함께 띄운다. 없으면 /lane_offset이 아예 나오지 않아
preflight가 FAIL 한다.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from main.control import (
    GUARDRAIL_PARAM_HELP,
    GUARDRAIL_PARAMS,
    GUARDRAIL_TUNABLES,
    STEERING_FILTER_PARAM_HELP,
    STEERING_FILTER_PARAMS,
    STEERING_FILTER_TUNABLES,
)

# ── 가드레일 이득 런치 인수 ──────────────────────────────────────────────────
# 이름/기본값/설명을 전부 main.control 에서 읽는다. 런치 파일에 기본값을 복사해
# 두면 소스 쪽만 고쳤을 때 조용히 갈라지기 때문이다.
#
# 여기서 지정하지 않아도 main_node 는 같은 기본값으로 뜨고, 주행 중에도 바꿀 수
# 있다 — 재빌드는 물론 재실행도 필요 없다:
#     ros2 param set /main_node guardrail_gain_deg 20.0
#     ros2 param get /main_node guardrail_gain_deg
def guardrail_launch_arguments():
    return [
        DeclareLaunchArgument(
            f'guardrail_{name}',
            default_value=str(GUARDRAIL_PARAMS[name]),
            description=GUARDRAIL_PARAM_HELP[name],
        )
        for name in GUARDRAIL_TUNABLES
    ]


def guardrail_node_parameters():
    return {
        f'guardrail_{name}': ParameterValue(
            LaunchConfiguration(f'guardrail_{name}'),
            value_type=type(GUARDRAIL_PARAMS[name]),
        )
        for name in GUARDRAIL_TUNABLES
    }


# 지터 억제(데드밴드/저역통과)도 같은 방식으로 연다. 이쪽은 main_node 가
# 파라미터 콜백을 갖고 있어 주행 중에도 바꿀 수 있다:
#     ros2 param set /main_node offset_deadband_px 12.0
# 값은 직선 구간에서 main/tools/curve_diag.py 로 offset 산포를 재고 정한다.
def steering_filter_launch_arguments():
    return [
        DeclareLaunchArgument(
            name,
            default_value=str(STEERING_FILTER_PARAMS[name]),
            description=STEERING_FILTER_PARAM_HELP[name],
        )
        for name in STEERING_FILTER_TUNABLES
    ]


def steering_filter_node_parameters():
    return {
        name: ParameterValue(
            LaunchConfiguration(name),
            value_type=type(STEERING_FILTER_PARAMS[name]),
        )
        for name in STEERING_FILTER_TUNABLES
    }



OFFLINE_MOTOR_TOPIC = "/kmu_main_offline/xycar_motor"


def generate_launch_description():
    live_drive = LaunchConfiguration("live_drive")
    show_debug = LaunchConfiguration("show_debug")
    udp_motor_bridge = LaunchConfiguration("udp_motor_bridge")

    arguments = [
        DeclareLaunchArgument(
            "live_drive",
            default_value="false",
            choices=("false", "true"),
            description=(
                "false isolates Main output; true starts camera/LiDAR inputs "
                "and publishes /xycar_motor (motor driver still required)"
            ),
        ),
        DeclareLaunchArgument("show_debug", default_value="false"),
        DeclareLaunchArgument(
            "udp_motor_bridge", default_value="true", choices=("false", "true"),
            description="Forward live lane-only motor output to the ROS1 UDP receiver",
        ),
        DeclareLaunchArgument(
            "pidnet_model",
            default_value=os.path.join(
                get_package_share_directory("segmentation_tools"),
                "model", "pidnet_s_best.pt",
            ),
            description="PIDNet-S checkpoint path",
        ),
        DeclareLaunchArgument(
            "lane_debug",
            default_value="true",
            choices=("false", "true"),
            description=(
                "통합 모니터 창 'Lane Drive Monitor' 표시 "
                "(CAMERA=세그멘테이션 오버레이 / MASK=마스크 단독 / "
                "VEHICLE=조향, 3패널 1창). "
                "이 프로파일은 차선 인지를 눈으로 보며 튜닝하는 용도라 기본이 true 다. "
                "영상 표시가 CPU를 쓰므로 기록 주행에서는 lane_debug:=false 로 끌 것."
            ),
        ),
        DeclareLaunchArgument(
            "lane_debug_detail",
            default_value="false",
            choices=("false", "true"),
            description=(
                "보조 차선 창까지 표시 (ROI Polygon / BEV-PIDNet-Center-Lane / "
                "Lane View + Offset). lane_debug:=true 일 때만 의미가 있다."
            ),
        ),
        DeclareLaunchArgument(
            "pidnet_lane_classes",
            default_value="[1]",
            description=(
                "lane_node 가 중앙선으로 쓸 PIDNet 클래스 "
                "(1=center_lane, 2=left_solid, 3=right_solid). lane_node는 선을 "
                "하나만 피팅하므로 기본값은 중앙선 단독인 [1] 이다."
            ),
        ),
        # /lane_path_preview 유효 조건. lane_node 는 생성자에서 한 번만 읽으므로
        # ros2 param set 으로는 못 바꾼다 — 여기서 넘겨야 한다.
        #
        # 실차 계측에서 confidence 가 390 프레임 내내 0 이었다. 어느 조건에서
        # 걸리는지 가르려면 하나씩 풀어 보는 수밖에 없어서 노출한다. 기본값은
        # lane_detection.cpp 의 declare_parameter 기본값과 같아, 지정하지 않으면
        # 거동이 지금과 정확히 같다.
        DeclareLaunchArgument(
            "path_preview_target_y_ratio", default_value="0.25",
            description=(
                "선행 목표점의 BEV y 비율(0=먼쪽 끝, 1=차량쪽). 이 y 를 슬라이딩 "
                "윈도우 중심이 감싸지 못하면 preview 가 통째로 무효가 된다. "
                "커브에서 중앙선이 위까지 안 올라오면 여기서 걸린다"
            ),
        ),
        DeclareLaunchArgument(
            "path_preview_min_windows", default_value="7",
            description="2차 피팅에 필요한 최소 윈도우 중심 개수",
        ),
        DeclareLaunchArgument(
            "path_preview_min_span_ratio", default_value="0.45",
            description="윈도우 중심이 덮어야 할 최소 BEV 높이 비율",
        ),
        DeclareLaunchArgument(
            "path_preview_max_rmse_px", default_value="25.0",
            description="2차 피팅 잔차 상한(BEV px)",
        ),
        DeclareLaunchArgument(
            "lane_reacquire_fallback",
            default_value="true",
            choices=("false", "true"),
            description=(
                "차선을 놓쳤을 때 corridor 밖 BEV 전체에서 재탐색한다. "
                "module_drive.py(production)는 이 값을 true 로 고정하므로, "
                "production 과 같은 조건으로 비교하려면 true 로 둔다. "
                "조향 방식만 A/B 로 보고 싶으면 false 로 꺼서 변수를 줄인다."
            ),
        ),
        DeclareLaunchArgument(
            "lane_guardrail",
            default_value="true",
            choices=("false", "true"),
            description=(
                "바깥 실선(left_solid/right_solid)에 가까워질수록 반대쪽으로 "
                "조향을 더한다. Pure Pursuit 단독은 조향이 38.9 에서 포화하므로 "
                "코너에서 차선을 벗어나도 더 꺾지 못한다. false 면 반발항이 "
                "정확히 0 이 되어 Pure Pursuit 단독 거동과 완전히 같다 — "
                "A/B 비교의 기준선으로 쓴다."
            ),
        ),
        DeclareLaunchArgument(
            "enter_motor_toggle",
            default_value="true",
            choices=("false", "true"),
            description=(
                "이 터미널에서 Enter 를 누를 때마다 모터 출력을 정지/주행 으로 "
                "바꾼다. 인지·FSM·조향은 계속 돌고 바퀴로 나가는 속도만 0 이 "
                "되므로, 차를 세운 채 차선 인식을 보며 튜닝할 수 있다. "
                "주행은 켜진 상태로 시작한다. 터미널 없이 실행하면(리다이렉트 "
                "등) 토글만 조용히 꺼지고 주행은 그대로다."
            ),
        ),
        DeclareLaunchArgument(
            "curve_preview_enabled",
            default_value="true",
            choices=("false", "true"),
            description=(
                "먼 곡선 목표점 선행조향. false면 기존 /lane_offset 단독 "
                "Pure Pursuit로 즉시 롤백한다."
            ),
        ),
        *guardrail_launch_arguments(),
        *steering_filter_launch_arguments(),
        # PIDNet 이 트랙 밖 오검출을 지울 때 쓰는 바깥 실선 검증. runner 생성자에서
        # 한 번만 읽으므로 ros2 param set 으로는 못 바꾼다 — 여기서 넘겨야 한다.
        # lane_node 가 "왜 피팅에 실패했는지" 를 분류해 발행한다. 계측 결과
        # 200px 이상 점프의 75%가 10프레임 이상 차선을 놓친 직후에 났으므로,
        # 점프 자체보다 "왜 놓치는가" 가 근본이다. 그걸 보려면 이게 필요하다.
        # production 은 false 라 publisher 조차 만들지 않는다.
        DeclareLaunchArgument(
            "lane_pipeline_diagnostics", default_value="false",
            choices=("false", "true"),
            description=(
                "/lane_detection/pipeline_diagnostics 발행. 피팅 실패 이유와 "
                "단계별 화소 수가 나온다. 진단 주행에서만 켠다"
            ),
        ),
        DeclareLaunchArgument(
            "rail_support_radius", default_value="7",
            description=(
                "left_solid/right_solid 성분 주변 몇 화소까지를 '붙어 있다'로 "
                "볼지. 0 이면 레일 검증을 끈다(중앙선 검증은 그대로)"
            ),
        ),
        DeclareLaunchArgument(
            "rail_min_support_ratio", default_value="0.15",
            description=(
                "테두리 중 주행면 비율이 이 값 미만인 실선은 트랙 밖 오검출로 "
                "보고 지운다. 중앙선(0.5)보다 훨씬 낮다 — 실선은 바깥쪽이 "
                "정당하게 background 라 진짜도 0.3~0.6 밖에 안 나온다"
            ),
        ),
    ]
    parameters = [{
        "mode": 1,
        "show_debug": show_debug,
        "lane_guardrail": ParameterValue(
            LaunchConfiguration("lane_guardrail"), value_type=bool
        ),
        "curve_preview_enabled": ParameterValue(
            LaunchConfiguration("curve_preview_enabled"), value_type=bool
        ),
        "enter_motor_toggle": ParameterValue(
            LaunchConfiguration("enter_motor_toggle"), value_type=bool
        ),
        **guardrail_node_parameters(),
        **steering_filter_node_parameters(),
    }]
    isolated_main = Node(
        package="main",
        executable="main_node",
        name="main_node",
        output="screen",
        parameters=parameters,
        remappings=[("xycar_motor", OFFLINE_MOTOR_TOPIC)],
        condition=UnlessCondition(live_drive),
    )
    live_main = Node(
        package="main",
        executable="main_node",
        name="main_node",
        output="screen",
        parameters=parameters,
        condition=IfCondition(live_drive),
    )
    live_motor_bridge = Node(
        package="main", executable="udp_motor_bridge", name="udp_motor_bridge",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", live_drive, "' == 'true' and '", udp_motor_bridge,
            "' == 'true'",
        ])),
    )
    resize_node = Node(
        package="image_resize",
        executable="resize_node",
        name="resize_node",
        output="screen",
    )
    pidnet_node = Node(
        package="segmentation_tools",
        executable="pidnet_inference",
        name="pidnet_inference_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("pidnet_model"),
            "input_topic": "/resized_image",
            "class_topic": "/pidnet_class_map",
            "device": "auto",
            "rail_support_radius": ParameterValue(
                LaunchConfiguration("rail_support_radius"), value_type=int
            ),
            "rail_min_support_ratio": ParameterValue(
                LaunchConfiguration("rail_min_support_ratio"), value_type=float
            ),
        }],
    )
    lane_node = Node(
        package="lane_detection",
        executable="lane_node",
        name="lane_node",
        output="screen",
        parameters=[{
            "debug_view": ParameterValue(
                LaunchConfiguration("lane_debug"), value_type=bool
            ),
            "debug_lane_view": ParameterValue(
                LaunchConfiguration("lane_debug_detail"), value_type=bool
            ),
            # lane_node 는 생성자에서 한 번만 읽는다(파라미터 콜백 없음).
            # 그래서 실행 후 ros2 param set 으로는 바꿀 수 없고 여기서 넘겨야 한다.
            "enable_reacquire_full_bev_fallback": ParameterValue(
                LaunchConfiguration("lane_reacquire_fallback"), value_type=bool
            ),
            # lane_node 가 /pidnet_class_map 에서 직접 중앙선을 뽑으므로,
            # pidnet 과 같은 클래스 목록을 받아야 의미가 어긋나지 않는다.
            "center_classes": ParameterValue(
                LaunchConfiguration("pidnet_lane_classes"), value_type=List[int]
            ),
            "path_preview_target_y_ratio": ParameterValue(
                LaunchConfiguration("path_preview_target_y_ratio"),
                value_type=float,
            ),
            "path_preview_min_windows": ParameterValue(
                LaunchConfiguration("path_preview_min_windows"), value_type=int
            ),
            "path_preview_min_span_ratio": ParameterValue(
                LaunchConfiguration("path_preview_min_span_ratio"),
                value_type=float,
            ),
            "path_preview_max_rmse_px": ParameterValue(
                LaunchConfiguration("path_preview_max_rmse_px"),
                value_type=float,
            ),
            "publish_pipeline_diagnostics": ParameterValue(
                LaunchConfiguration("lane_pipeline_diagnostics"),
                value_type=bool,
            ),
        }],
        remappings=[("/mode_info", "/internal/lane_command")],
    )
    preflight = Node(
        package="main",
        executable="kmu_preflight",
        name="lane_only_preflight",
        output="screen",
        parameters=[{
            "required_topics": ["/lane_offset", "/scan"],
            "motor_output_topic": OFFLINE_MOTOR_TOPIC,
            "require_motor_subscriber": False,
        }],
        condition=UnlessCondition(live_drive),
    )
    live_preflight = Node(
        package="main",
        executable="kmu_preflight",
        name="lane_only_live_preflight",
        output="screen",
        parameters=[{
            "required_topics": ["/lane_offset", "/scan"],
            "motor_output_topic": "/xycar_motor",
            "require_motor_subscriber": True,
        }],
        condition=IfCondition(live_drive),
    )
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("xycar_cam"),
                "launch/xycar_cam.launch.py",
            )
        ),
        condition=IfCondition(live_drive),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("xycar_lidar"),
                "launch/xycar_lidar.launch.py",
            )
        ),
        condition=IfCondition(live_drive),
    )
    return LaunchDescription([
        *arguments,
        isolated_main,
        live_main,
        live_motor_bridge,
        resize_node,
        pidnet_node,
        lane_node,
        preflight,
        live_preflight,
        camera,
        lidar,
    ])
