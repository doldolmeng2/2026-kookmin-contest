"""터미널 Enter 로 내리는 모터 출력 게이트.

게이트는 /xycar_motor 로 나가는 **속도만** 막는다. FSM·인지·조향은 평소대로
돌아야 한다 — 차를 세운 채 차선 인식을 보며 튜닝하는 것이 이 기능의 목적이다.
"""

import threading

import pytest
import rclpy

from main.main import MainNode


class MotorRecorder:
    """motor_pub.publish 를 가로채 발행된 [각도, 속도] 를 모은다."""

    def __init__(self, node):
        self.commands = []
        node.motor_pub.publish = lambda msg: self.commands.append(list(msg.data))

    @property
    def last(self):
        return self.commands[-1]


def _node_holding_at_speed(speed):
    """HOLD 로 감속 중인 노드. 게이트가 없으면 속도가 0 이 아니다."""
    node = MainNode()
    node._now_seconds = lambda: 100.0
    node.now_angle = 12.5
    node.now_speed = speed
    return node


def test_gate_starts_open_and_the_stdin_thread_stays_off_by_default():
    """기본값은 '주행'이고, 파라미터를 켜지 않으면 터미널 스레드는 뜨지 않는다.

    기본으로 터미널을 읽으면 테스트 수행 중 제어 터미널 입력을 가로챈다.
    """
    rclpy.init()
    try:
        node = _node_holding_at_speed(0.0)
        assert node._motor_gate.is_set() is True
        assert node.get_parameter("enter_motor_toggle").value is False
        assert not [
            thread for thread in threading.enumerate()
            if thread.name == "enter_motor_toggle"
        ]
    finally:
        rclpy.shutdown()


def test_open_gate_publishes_the_shaped_speed_unchanged():
    rclpy.init()
    try:
        node = _node_holding_at_speed(21.5)
        recorder = MotorRecorder(node)
        node.control_cycle()

        angle, speed = recorder.last
        assert speed > 0.0                      # 게이트가 열려 있으면 그대로 나간다
        # 발행 값은 float32 로 잘리므로 근사 비교한다.
        assert speed == pytest.approx(node.now_speed, rel=1e-6)
        assert angle == pytest.approx(node.now_angle, rel=1e-6)
    finally:
        rclpy.shutdown()


def test_closed_gate_zeroes_speed_but_keeps_steering():
    """속도만 0 이 된다. 조향은 제어가 계산한 값 그대로 나가야 한다.

    코너에서 멈출 때 앞바퀴를 0 으로 펴 버리면 다시 출발할 때 차가 코스 밖을
    향한다.
    """
    rclpy.init()
    try:
        node = _node_holding_at_speed(21.5)
        recorder = MotorRecorder(node)
        node._set_motor_enabled(False)
        node.control_cycle()

        angle, speed = recorder.last
        assert speed == 0.0
        assert angle == node.now_angle == 12.5
    finally:
        rclpy.shutdown()


def test_closed_gate_resets_the_ramp_so_reopening_starts_from_zero():
    """다시 켠 순간 멈추기 직전 속도로 튀면 안 된다."""
    rclpy.init()
    try:
        node = _node_holding_at_speed(21.5)
        recorder = MotorRecorder(node)
        node._set_motor_enabled(False)
        node.control_cycle()
        assert node.now_speed == 0.0

        node._set_motor_enabled(True)
        node.control_cycle()
        # 램프를 0 부터 다시 탄다. 21.5 로 되돌아가지 않는다.
        assert recorder.last[1] < 1.0
    finally:
        rclpy.shutdown()


def test_toggling_flips_the_gate_and_repeating_a_state_is_a_no_op():
    rclpy.init()
    try:
        node = _node_holding_at_speed(0.0)
        assert node._motor_gate.is_set() is True

        node._set_motor_enabled(False)
        assert node._motor_gate.is_set() is False
        node._set_motor_enabled(False)          # 같은 상태 재지정은 무시
        assert node._motor_gate.is_set() is False

        node._set_motor_enabled(True)
        assert node._motor_gate.is_set() is True
    finally:
        rclpy.shutdown()


def test_toggle_loop_reads_the_controlling_terminal_not_stdin():
    """`ros2 launch` 는 자식 stdin 을 파이프로 바꾼다. /dev/tty 를 열어야 한다.

    sys.stdin 을 읽도록 되돌아가면 런치로 띄웠을 때 Enter 가 영원히 오지 않는다
    — 조용히 동작만 안 하는 회귀라 소스로 못을 박아 둔다.
    """
    import inspect

    # 주석·docstring 에는 sys.stdin 을 쓰지 않는 **이유**가 적혀 있다. 그 설명이
    # 검사에 걸리지 않도록 docstring 과 주석을 떼고 실행되는 줄만 본다.
    source = inspect.getsource(MainNode._run_motor_toggle_loop)
    code = "\n".join(
        line for line in source.split('"""')[2].splitlines()
        if not line.lstrip().startswith("#")
    )
    assert '"/dev/tty"' in code
    assert "sys.stdin" not in code
