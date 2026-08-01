# ─────────────────────────────────────────────────────────────────────────────
# main.py
#
# 역할: 자율주행 전체 상태 머신 및 제어 루프 (메인 오케스트레이터 노드)
#
# 모드 전이 순서:
#   TRAFFIC_WAIT(0) → 초록불 감지 → RUBBERCONE_DRIVE(1)
#   RUBBERCONE_DRIVE  → end_flag=1 → RUBBERCONE_END(2)
#   RUBBERCONE_END    → 1.4초 경과 → BEFORE(4)
#   BEFORE            → 추월 조건 충족 → LANE_DRIVE(3)
#   LANE_DRIVE        → 객체 박스 ≥ 1900px² → CHANGE_LANE(5)
#   CHANGE_LANE       → 차선 변경 완료 → BEFORE(4)
#
# 구독:
#   rubbercone_info   (std_msgs/Int32MultiArray)  - [오프셋, end_flag, 신뢰도]
#   lane_offset       (std_msgs/Int16)            - 차선 중심 오프셋
#   object_info       (std_msgs/Float32MultiArray)- 장애물 10필드
#   traffic_detection (std_msgs/Bool)             - 초록불 감지 여부
#   joy               (sensor_msgs/Joy)           - Xbox 컨트롤러 입력
#   xycar_ultrasonic  (std_msgs/Int32MultiArray)  - 초음파 거리 배열
#
# 발행:
#   xycar_motor (std_msgs/Float32MultiArray) - [조향각, 속도]
#   mode_info   (std_msgs/Int32MultiArray)   - [현재 모드, 현재 차선]
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16, Bool, Float32MultiArray
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import numpy as np

from main.control import Controller

# ── 주행 모드 상수 ───────────────────────────────────────────────────────────
TRAFFIC_WAIT     = 0  # 신호 대기: 완전 정지
RUBBERCONE_DRIVE = 1  # 라바콘 구간 주행
RUBBERCONE_END   = 2  # 라바콘 종료 후 차선 진입 (고정 조향)
LANE_DRIVE       = 3  # 차선 주행
BEFORE           = 4  # 장애물 접근 대기 (추월 판단 중)
CHANGE_LANE      = 5  # 차선 변경 중

# 라바콘 구간은 LiDAR 프레임마다 경로 신뢰도가 달라질 수 있다. 목표 속도 변화는
# 아래 단계로 제한해, 한 프레임의 경계 누락이 모터 명령의 급변으로 이어지지 않게 한다.
# 0.10(초당 5.0)에서는 순항 11.0까지 2.2초가 걸려, 4~6초짜리 라바콘 구간의
# 절반을 가속에만 썼다(실주행 bag 측정 1.86초). 0.25면 0.9초로 줄어든다.
RUBBERCONE_ACCEL_STEP = 0.25  # 20 ms마다, 실제 모터 속도 단위 (초당 12.5)
RUBBERCONE_DECEL_STEP = 0.35  # 급조향 때는 가속보다 빠르게 감속 (초당 17.5)
# LiDAR 경로는 약 10 Hz로 갱신된다. 한 스캔의 경계 선택이 바뀌어도 실제
# 조향 명령이 즉시 반대 끝까지 가지 않도록, 50 Hz 제어 주기마다 변화량을 제한한다.
RUBBERCONE_STEERING_STEP = 3.0  # 20 ms마다 조향각 최대 변화량 (deg)


class MainNode(Node):
    """
    자율주행 메인 노드.
    센서 데이터를 수집하고 상태 머신으로 주행 모드를 결정한 뒤
    Controller를 통해 조향각/속도를 계산하여 모터에 발행한다.
    제어 루프는 20ms 타이머(약 50Hz)로 동작한다.
    """

    def __init__(self):
        super().__init__('main_node')

        # ── 주행 상태 ────────────────────────────────────────────────────────
        self.declare_parameter('mode', TRAFFIC_WAIT)
        self.mode             = self.get_parameter('mode').value
        self.declare_parameter('show_debug', False)
        self.show_debug       = self.get_parameter('show_debug').value
        self.lane             = 1        # 현재 주행 차선: 0=Lane1(왼쪽), 1=Lane2(오른쪽)
        self.test_mode        = False    # True이면 자동 모드 전환 비활성화 (수동 디버그용)

        # ── 라바콘 종료 타이머 ───────────────────────────────────────────────
        self.rubbercone_end_time = None  # RUBBERCONE_END 진입 시각
        self.into_lane_timer     = 1.4  # 라바콘 종료 후 차선 진입까지 대기 시간(초)

        # ── 차선 진입 후 초기 속도 제한 ────────────────────────────────────
        # TRAFFIC_WAIT 이후 첫 번째 control_cycle 시점을 기록하여
        # LANE_DRIVE 진입 3초 이내에는 속도를 5.0으로 제한한다.
        self.lane_drive_started    = False
        self.lane_drive_start_time = None

        # ── 속도 부드러운 증가 ───────────────────────────────────────────────
        self.now_speed = 0  # 실제 발행 속도 (목표 속도까지 0.1씩 증가)
        self.now_angle = 0.0  # 실제 발행 조향각 (라바콘에서 변화율 제한 적용)
        self.last_debug_time = None

        # ── 추월 완료 판정용 카운터 ─────────────────────────────────────────
        # 차선 변경 후 반대쪽 초음파가 25cm 이하를 10회 이상 감지하면 추월 완료
        self.avoid_cnt = 0

        # ── 컨트롤러 초기화 ──────────────────────────────────────────────────
        self.controller = Controller(self)

        # ── QoS 프로파일 (센서/수치 토픽용) ─────────────────────────────────
        # Best Effort + Volatile: 최신 값 우선, 지연 내성 없음
        qos_fast = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_command = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── 발행자 ───────────────────────────────────────────────────────────
        # 모터 명령도 오래된 값이 쌓이지 않도록 최신 한 개만 보관한다.
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', qos_command)
        self.mode_pub  = self.create_publisher(Int32MultiArray,  'mode_info',   qos_fast)

        # ── 구독자 ───────────────────────────────────────────────────────────
        self.create_subscription(Int32MultiArray,  'rubbercone_info',  self.rubbercone_callback,   qos_fast)
        self.create_subscription(Int16,            'lane_offset',      self.lane_offset_callback,  qos_fast)
        self.create_subscription(Float32MultiArray,'object_info',      self.object_info_callback,  qos_fast)
        self.create_subscription(Bool,             'traffic_detection',self.traffic_callback,      qos_fast)
        self.create_subscription(Joy,              'joy',              self.joy_callback,           10)
        self.create_subscription(Int32MultiArray,  'xycar_ultrasonic', self.ultrasonic_callback,    10)

        # ── Xbox 컨트롤러 디바운스 상태 ─────────────────────────────────────
        self.prev_x = 0  # 이전 프레임의 X 버튼 상태
        self.prev_b = 0  # 이전 프레임의 B 버튼 상태

        # ── 센서 수신 변수 초기값 ────────────────────────────────────────────
        self.rubbercone_offset = 0       # 라바콘 조향 오프셋 (px)
        self.end_flag          = 0       # 라바콘 종료 플래그 (0/1)
        self.rubbercone_confidence = 0   # 라바콘 경로 추정 신뢰도 (0~100)
        self.lane_offset       = 0       # 차선 중심 오프셋 (px)
        self.traffic_green     = False   # 초록 신호등 감지 여부

        # 초음파: left/right (초음파 배열에서 인덱스 0번, 4번)
        self.left  = float('inf')  # 왼쪽 초음파 거리 (cm)
        self.right = float('inf')  # 오른쪽 초음파 거리 (cm)

        # 장애물 정보 10필드 (object_detection 노드에서 발행)
        self.obj_exists  = 0.0          # 장애물 존재 여부 (0/1)
        self.object_dist = float('inf') # 장애물까지 최소 거리 (m)
        self.obj_angle   = 0.0          # 장애물 방위각 (rad)
        self.obj_span    = 0.0          # 장애물 각도 폭 (rad)
        self.obj_cluster = 0.0          # LiDAR 클러스터 포인트 수
        self.box_size    = 0.0          # YOLO 바운딩 박스 면적 (px²)
        self.box_cx      = float('nan') # 박스 중심 x (px)
        self.box_cy      = float('nan') # 박스 중심 y (px)
        self.box_dx      = float('nan') # 박스 중심 x 편차 (px, 차선 중심 기준)
        self.car_lane    = -1           # 장애물이 위치한 차선: -1=미정, 0=중앙, 1=L1, 2=L2

        # ── 20ms 제어 타이머 (~50Hz) ─────────────────────────────────────────
        self.create_timer(0.02, self.control_cycle)

    # ── 구독 콜백 ─────────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        """
        Xbox 컨트롤러 입력으로 수동 모드 전환을 처리한다 (디버그/테스트 용도).
          X 버튼 (버튼 2) 상승 에지: mode-- (이전 모드로)
          B 버튼 (버튼 1) 상승 에지: mode++ (다음 모드로)
        라이징 에지 감지로 버튼을 누르는 동안 연속 전환을 방지한다.
        """
        x = msg.buttons[2]
        b = msg.buttons[1]

        if x == 1 and self.prev_x == 0:
            # X 버튼: 모드 감소 + 라바콘 상태 초기화
            self.end_flag          = 0
            self.rubbercone_end_time = None
            self.mode = max(TRAFFIC_WAIT, self.mode - 1)
            self.get_logger().info(f"Mode-- -> {self.mode}")

        if b == 1 and self.prev_b == 0:
            # B 버튼: 모드 증가
            self.mode = min(CHANGE_LANE, self.mode + 1)
            self.get_logger().info(f"Mode++ -> {self.mode}")

        self.prev_x = x
        self.prev_b = b

    def rubbercone_callback(self, msg: Int32MultiArray):
        """라바콘 노드의 조향 오프셋, 종료 플래그, 경로 신뢰도를 수신한다."""
        if len(msg.data) >= 2:
            self.rubbercone_offset = msg.data[0]
            self.end_flag          = msg.data[1]
            # 구 버전 노드(2필드)와도 호환되도록 중간 신뢰도를 사용한다.
            self.rubbercone_confidence = (
                max(0, min(100, int(msg.data[2]))) if len(msg.data) >= 3 else 50
            )

    def lane_offset_callback(self, msg: Int16):
        """차선 검출 노드에서 발행한 차선 중심 오프셋을 수신한다."""
        self.lane_offset = msg.data

    def object_info_callback(self, msg: Float32MultiArray):
        """
        장애물 검출 노드에서 발행한 10개 필드의 장애물 정보를 파싱한다.
        필드 순서: [exists, min_dist, angle, span, cluster_size,
                    box_size, box_cx, box_cy, dx, car_lane]
        """
        d = msg.data
        self.obj_exists  = float(d[0])  # 장애물 존재 여부 (0/1)
        self.object_dist = float(d[1])  # 최소 거리 (m)
        self.obj_angle   = float(d[2])  # 방위각 (rad)
        self.obj_span    = float(d[3])  # 각도 폭 (rad)
        self.obj_cluster = float(d[4])  # 클러스터 크기
        self.box_size    = float(d[5])  # 바운딩 박스 면적 (px²)
        self.box_cx      = float(d[6])  # 박스 중심 x (px)
        self.box_cy      = float(d[7])  # 박스 중심 y (px)
        self.box_dx      = float(d[8])  # 박스 중심 x 편차 (px)
        self.car_lane    = int(d[9])    # 장애물 차선: 1=L1, 2=L2, 0=중앙, -1=미정

    def traffic_callback(self, msg: Bool):
        """신호등 검출 노드에서 초록불 감지 여부를 수신한다."""
        self.traffic_green = msg.data

    def ultrasonic_callback(self, msg: Int32MultiArray):
        """
        초음파 센서 배열에서 왼쪽(인덱스 0)과 오른쪽(인덱스 4) 거리를 수신한다.
        단위: cm. 추월 완료 판단(is_pass_comp)에 사용된다.
        """
        data = msg.data
        if len(data) > 5:
            self.left  = data[0]
            self.right = data[4]

    # ── 제어 루프 (20ms, ~50Hz) ───────────────────────────────────────────────

    def control_cycle(self):
        """
        메인 제어 루프. 매 20ms마다 호출된다.
          1) 자동 모드 전환 판정 (test_mode=False일 때)
          2) 오프셋 선택 (라바콘/차선)
          3) Controller를 통해 조향각·속도 계산
          4) 속도 부드러운 증가 적용
          5) LANE_DRIVE 초기 3초 속도 제한 적용
          6) 모터 및 모드 발행
          7) 디버그 상태 창 갱신
        """
        now = self.get_clock().now()

        # 라바콘 종료 후 경과 시간 계산
        elapsed = (
            (now - self.rubbercone_end_time).nanoseconds / 1e9
            if self.rubbercone_end_time is not None else 0.0
        )

        # ── 자동 모드 전환 ───────────────────────────────────────────────────
        if not self.test_mode:
            if self.mode == TRAFFIC_WAIT and self.traffic_green:
                # 초록불 감지 → 라바콘 구간 주행 시작
                self.mode = RUBBERCONE_DRIVE
                self.get_logger().info("초록불 감지 -> 라바콘 모드로 변경")

            elif self.mode == RUBBERCONE_DRIVE and self.end_flag == 1:
                # 라바콘 구간 종료 → 고정 조향으로 차선 진입
                self.mode = RUBBERCONE_END
                self.rubbercone_end_time = now
                self.get_logger().info("라바콘 종료 차선 진입")

            elif self.mode == RUBBERCONE_END and elapsed > self.into_lane_timer:
                # 차선 진입 대기 시간 초과 → 장애물 접근 대기 모드
                self.mode = BEFORE

            elif self.mode == BEFORE:
                if self.is_pass_comp():
                    # 추월 조건 충족 → 차선 주행 모드
                    self.mode = LANE_DRIVE
                    self.get_logger().info("추월 전 상태")

            elif self.mode == LANE_DRIVE:
                if self.box_size > 1900:
                    # 장애물 박스가 충분히 크면 차선 변경 시작
                    self.mode = CHANGE_LANE
                    self.lane = 1 - self.lane  # 차선 토글: Lane1 ↔ Lane2
                    self.get_logger().info("객체 박스 감지, 차선 변경 모드 전환")

            elif self.mode == CHANGE_LANE and self.is_change_end():
                # 차선 변경 완료 → 장애물 접근 대기 모드로 복귀
                self.mode = BEFORE
                self.get_logger().info("차선 변경 완료")

        # ── 오프셋 선택 ──────────────────────────────────────────────────────
        # 라바콘 구간에서는 LiDAR 기반 오프셋, 나머지는 카메라 기반 차선 오프셋 사용
        offset = self.rubbercone_offset if self.mode == RUBBERCONE_DRIVE else self.lane_offset

        # ── 조향각·속도 계산 ─────────────────────────────────────────────────
        cone_confidence = (
            self.rubbercone_confidence
            if self.mode == RUBBERCONE_DRIVE else 100
        )
        self.controller.update(
            self.mode, offset, self.object_dist, cone_confidence)
        target_angle = self.controller.get_angle()
        target_speed = self.controller.get_speed() / 2

        # 라바콘에서는 새 scan이 들어온 한 프레임에 최대 조향으로 튀지 않도록
        # 실제 모터 명령을 램프 처리한다. 다른 모드의 기존 즉시 조향 동작은 보존한다.
        if self.mode == RUBBERCONE_DRIVE:
            angle_delta = target_angle - self.now_angle
            self.now_angle += max(
                -RUBBERCONE_STEERING_STEP,
                min(RUBBERCONE_STEERING_STEP, angle_delta),
            )
        else:
            self.now_angle = target_angle
        angle = self.now_angle

        # ── 차선 진입 초기 속도 제한 ────────────────────────────────────────
        # TRAFFIC_WAIT → 이후 첫 control_cycle에서 시작 시각을 기록한다.
        # LANE_DRIVE 진입 후 3초간 속도를 5.0으로 제한하여 급출발을 방지한다.
        if not self.lane_drive_started:
            self.lane_drive_started    = True
            self.lane_drive_start_time = now
        if self.mode == TRAFFIC_WAIT:
            self.lane_drive_started = False

        if self.mode == LANE_DRIVE and self.lane_drive_start_time is not None:
            elapsed_lane = (now - self.lane_drive_start_time).nanoseconds / 1e9
            if elapsed_lane < 3.0:
                target_speed = min(target_speed, 5.0)

        # 라바콘은 경계 신뢰도가 스캔마다 달라질 수 있으므로 목표 속도를 양방향으로
        # 부드럽게 따라간다. 다른 모드는 기존처럼 감속을 즉시 반영한다.
        if self.mode == RUBBERCONE_DRIVE:
            if self.now_speed < target_speed:
                self.now_speed = min(
                    target_speed, self.now_speed + RUBBERCONE_ACCEL_STEP)
            else:
                self.now_speed = max(
                    target_speed, self.now_speed - RUBBERCONE_DECEL_STEP)
        elif self.now_speed < target_speed:
            self.now_speed = min(target_speed, self.now_speed + 0.1)
        else:
            self.now_speed = target_speed

        # ── 모터 발행 ────────────────────────────────────────────────────────
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(self.now_speed)]
        self.motor_pub.publish(motor_msg)

        # ── 모드 발행 ────────────────────────────────────────────────────────
        mode_msg = Int32MultiArray()
        mode_msg.data = [self.mode, self.lane]
        self.mode_pub.publish(mode_msg)

        self._update_debug_window(now, angle, self.now_speed, offset)

    def _update_debug_window(self, now, angle: float, speed: float, offset: int):
        """선택적 상태 창을 10 Hz로만 갱신해 제어 타이머를 막지 않는다."""
        if not self.show_debug:
            return

        if self.last_debug_time is not None:
            elapsed = (now - self.last_debug_time).nanoseconds / 1e9
            if elapsed < 0.1:
                return
        self.last_debug_time = now

        # OpenCV 창 렌더링은 실차 제어 경로와 분리한다.
        log_img = np.zeros((320, 640, 3), dtype=np.uint8)
        mode_map = {
            TRAFFIC_WAIT:     'TRAFFIC_WAIT',
            RUBBERCONE_DRIVE: 'RUBBERCONE_DRIVE',
            RUBBERCONE_END:   'RUBBERCONE_END',
            LANE_DRIVE:       'LANE_DRIVE',
            BEFORE:           'BEFORE',
            CHANGE_LANE:      'CHANGE_LANE',
        }
        lines = [
            f"Mode: {mode_map.get(self.mode, 'UNKNOWN')}",
            f"Angle: {angle:.1f}",
            f"Speed: {speed:.1f}",
            f"Offset: {offset}",
            f"Rubber confidence: {self.rubbercone_confidence}%",
            f"{'Lane 1' if self.lane == 0 else 'Lane 2'}",
            f"{'Rubber End' if self.end_flag == 1 else 'Rubber Not End'}",
            f"Object dist: {self.object_dist:.2f} m",
            f"Box: {self.box_size:.0f}px^2  car_lane={self.car_lane}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(log_img, text, (10, 30 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow('Status', log_img)
        cv2.waitKey(1)

    # ── 모드 전환 조건 판정 ───────────────────────────────────────────────────

    def is_change_end(self) -> bool:
        """
        차선 변경 완료 여부를 반환한다.
        현재는 항상 True를 반환하므로 CHANGE_LANE → BEFORE 전환이 즉시 이루어진다.
        """
        return True

    def is_pass_comp(self) -> bool:
        """
        추월 완료 여부를 판정한다.
        현재 차선과 반대쪽 초음파 센서가 25cm 이내를 10회 이상 감지하면
        장애물을 추월한 것으로 판단하고 True를 반환한다.

        로직:
          - Lane1(lane=0) 주행 중: 오른쪽(right) 초음파 < 25cm 감지
          - Lane2(lane=1) 주행 중: 왼쪽(left) 초음파 < 25cm 감지
        """
        if self.lane == 0 and self.right < 25:
            self.avoid_cnt += 1
            print("왼쪽 감지  카운트 :", self.avoid_cnt)
        elif self.lane == 1 and self.left < 25:
            self.avoid_cnt += 1
            print("오른쪽 감지  카운트 :", self.avoid_cnt)

        if self.avoid_cnt > 10:
            self.avoid_cnt = 0
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
