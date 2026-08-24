# ─────────────────────────────────────────────────────────────────────────────
# bag_motion_replay.launch.py
#
# 역할: 녹화된 주행 명령 스트림을 기록된 내용·타이밍 그대로 다시 발행한다.
#
# 기본값은 VESC 명령 계층(`/commands/servo/position`, `/commands/motor/speed`)이며
# 조이콘에서 나온 `/joy`·`/xycar_motor` 는 발행하지 않는다. 조이콘 토픽을 쓰려면
# allow_joycon:=true 를 명시해야 한다.
#
#   ros2 launch bag_motion_replay bag_motion_replay.launch.py
#   ros2 launch bag_motion_replay bag_motion_replay.launch.py topic_prefix:=/selftest
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_BAG = '~/my_rosbag/rosbag2_2026_08_13-13_33_23'

# (이름, 기본값, 타입, 설명)
ARGUMENTS = (
    ('bag', DEFAULT_BAG, str, 'rosbag2 디렉터리 경로'),
    ('cue', '', str, '미리 만든 cue 파일 경로 (비우면 캐시에서 찾거나 새로 만든다)'),
    ('rebuild_cue', 'false', bool, 'cue 캐시를 무시하고 다시 만든다'),
    ('topic_set', 'actuation', str, 'actuation | motor | full | sensors | all'),
    ('allow_joycon', 'false', bool, '/joy·/xycar_motor 도 재생할지 여부'),
    ('topic_prefix', '', str, '발행 토픽 앞에 붙일 접두어 (실차 토픽을 건드리지 않고 시험할 때)'),
    ('rate', '1.0', float, '재생 배속 (1.0 이 기록 그대로)'),
    ('start_offset', '0.0', float, '재생 시작 위치 [s]'),
    ('end_offset', '-1.0', float, '재생 종료 위치 [s], -1 이면 끝까지'),
    ('loop', '1', int, '반복 횟수, -1 이면 중단 요청까지 무한 반복'),
    ('loop_gap', '0.0', float, '반복 사이 간격 [s]'),
    ('timing_mode', 'wall', str, 'wall | sim (sim 은 /clock 을 직접 몰아 오차 0)'),
    ('spin_margin_ms', '1.5', float, '마감 직전 busy-wait 구간 [ms]'),
    ('realtime_priority', '0', int, '0 보다 크면 SCHED_FIFO 를 요청한다'),
    ('wait_for_subscribers', 'false', bool, '구독자가 붙을 때까지 기다렸다 시작한다'),
    ('start_delay', '1.0', float, '발행 시작 전 대기 [s]'),
    ('safe_stop_on_abort', 'true', bool, '강제 중단 시 정지 명령을 한 번 보낸다'),
    ('report_path', '', str, '실행 리포트를 저장할 JSON 경로'),
    ('fail_on_timing_error_ms', '0.0', float, '이 값을 넘는 오차가 나오면 실패로 처리 (0=검사 안 함)'),
)


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in ARGUMENTS
    ]

    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in ARGUMENTS
    }

    replay_node = Node(
        package='bag_motion_replay',
        executable='replay_node',
        name='bag_motion_replay',
        output='screen',
        emulate_tty=True,
        parameters=[parameters],
    )

    return LaunchDescription(declarations + [replay_node])
