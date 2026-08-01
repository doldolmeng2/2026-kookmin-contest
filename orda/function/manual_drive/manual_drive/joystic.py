#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# joystic.py
#
# 역할: Xbox 컨트롤러(조이스틱) 입력을 Xycar 모터 명령으로 변환하는 수동 주행 노드
#
# 구독: /joy (sensor_msgs/Joy) - 조이스틱 축/버튼 상태
# 발행: xycar_motor (std_msgs/Float32MultiArray) - [조향각, 속도]
#
# 매핑:
#   axes[0] × -100 → 조향각 (왼쪽 스틱 좌우)
#   axes[4] ×   50 → 속도   (오른쪽 스틱 상하)
#
# 10Hz (0.1초 타이머)로 최신 축 값을 주기적으로 발행한다.
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class JoyToMotor(Node):
    """
    조이스틱 입력을 모터 명령으로 변환하는 노드.
    joy 토픽을 구독하고, 10Hz 타이머에서 최신 축 값을 모터 토픽으로 발행한다.
    """

    def __init__(self):
        super().__init__('joy_to_motor')

        # 조이스틱 축 현재값 (joy_callback에서 갱신)
        self.axis_0 = 0.0  # 왼쪽 스틱 좌우 (조향)
        self.axis_4 = 0.0  # 오른쪽 스틱 상하 (속도)
        self.last_speed = 0.0 #이전 속도 (브레이킹 감지용)

        # 모터 명령 메시지 (재사용)
        self.motor_msg = Float32MultiArray()

        # joy 구독: 조이스틱 축/버튼 상태 수신
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # 모터 발행: xycar_motor [조향각, 속도]
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 10)

        # 20Hz 타이머로 주기적 발행
        self.create_timer(0.05, self.publish_motor)

        self.get_logger().info("Joy → Motor 노드 시작 (20Hz)")

    def joy_callback(self, msg: Joy):
        """조이스틱 축 값을 갱신한다. axes[0]: 조향, axes[4]: 속도."""
        if len(msg.axes) > 4:
            self.axis_0 = msg.axes[0]
            self.axis_4 = msg.axes[4]

    def publish_motor(self):
        """
        최신 축 값으로 조향각과 속도를 계산하여 xycar_motor에 발행한다.
          조향각 = axes[0] × -100  (부호 반전: 왼쪽 스틱 왼쪽 → 양수 조향)
          속도   = axes[4] ×   50
        """
        angle = -100 * self.axis_0
        speed =   7.5 * self.axis_4
        self.last_speed = speed

        self.motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(self.motor_msg)
        print(f"[PUBLISH] Angle: {angle:.2f}, Speed: {speed:.2f}", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()