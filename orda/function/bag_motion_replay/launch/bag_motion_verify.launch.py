# ─────────────────────────────────────────────────────────────────────────────
# bag_motion_verify.launch.py
#
# 역할: 재생 노드와 검증 노드를 함께 띄워, 발행된 내용과 타이밍이 녹화본과
#       일치하는지 실제 DDS 경로로 측정한다.
#
# 기본 topic_prefix 가 /selftest 라 실차 토픽에는 아무것도 나가지 않는다.
# 실차 토픽 그대로 검증하려면 topic_prefix:='' 로 준다.
#
#   ros2 launch bag_motion_replay bag_motion_verify.launch.py
#   ros2 launch bag_motion_replay bag_motion_verify.launch.py duration:=20.0
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'

SHARED = (
    ('bag', DEFAULT_BAG, str, 'rosbag2 디렉터리 경로'),
    ('cue', '', str, '미리 만든 cue 파일 경로'),
    ('topic_set', 'actuation', str, 'actuation | motor | full | sensors | all'),
    ('allow_joycon', 'false', bool, '/joy·/xycar_motor 도 재생할지 여부'),
    ('topic_prefix', '/selftest', str, '재생·검증이 함께 쓰는 토픽 접두어'),
)

REPLAY_ONLY = (
    ('rate', '1.0', float, '재생 배속'),
    ('start_offset', '0.0', float, '재생 시작 위치 [s]'),
    ('end_offset', '-1.0', float, '재생 종료 위치 [s], -1 이면 끝까지'),
    ('timing_mode', 'wall', str, 'wall | sim'),
    ('spin_margin_ms', '1.5', float, '마감 직전 busy-wait 구간 [ms]'),
    ('realtime_priority', '0', int, '0 보다 크면 SCHED_FIFO 를 요청한다'),
    ('replay_report_path', '', str, '재생 리포트 JSON 경로'),
)

VERIFY_ONLY = (
    ('idle_timeout', '5.0', float, '마지막 메시지 이후 이만큼 조용하면 채점한다 [s]'),
    ('verify_report_path', '', str, '검증 리포트 JSON 경로'),
)


def _declarations(groups):
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for group in groups
        for name, default, _, description in group
    ]


def _parameters(group):
    return {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in group
    }


def generate_launch_description():
    shared = _parameters(SHARED)

    verify_parameters = dict(shared)
    verify_parameters['idle_timeout'] = ParameterValue(
        LaunchConfiguration('idle_timeout'), value_type=float
    )
    verify_parameters['report_path'] = ParameterValue(
        LaunchConfiguration('verify_report_path'), value_type=str
    )

    replay_parameters = dict(shared)
    for name, _, value_type, _ in REPLAY_ONLY:
        if name == 'replay_report_path':
            continue
        replay_parameters[name] = ParameterValue(
            LaunchConfiguration(name), value_type=value_type
        )
    replay_parameters['report_path'] = ParameterValue(
        LaunchConfiguration('replay_report_path'), value_type=str
    )
    replay_parameters['wait_for_subscribers'] = True
    replay_parameters['start_delay'] = 1.5
    replay_parameters['safe_stop_on_abort'] = False

    verify_node = Node(
        package='bag_motion_replay',
        executable='verify_node',
        name='bag_motion_verify',
        output='screen',
        emulate_tty=True,
        parameters=[verify_parameters],
    )

    replay_node = Node(
        package='bag_motion_replay',
        executable='replay_node',
        name='bag_motion_replay',
        output='screen',
        emulate_tty=True,
        parameters=[replay_parameters],
    )

    return LaunchDescription(
        _declarations((SHARED, REPLAY_ONLY, VERIFY_ONLY)) + [verify_node, replay_node]
    )
