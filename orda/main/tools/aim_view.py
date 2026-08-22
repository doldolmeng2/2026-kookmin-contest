#!/usr/bin/env python3
"""lane_node 가 "중앙선을 어디에 두려고 하는지" 를 카메라 영상 위에 그린다.

/lane_offset 은 BEV 좌표에서 (중앙선 위치 - 기준선) 이다. 그 기준선이
center_reference_lane_* 인데, 값이 BEV 폭 비율이라 카메라 화면 어디에 해당하는지
눈으로 확인할 방법이 없었다. 차가 한쪽으로 치우쳐 간다면 이 기준선이 실제
주행선과 어긋난 것이므로, 먼저 보이게 만든다.

    python3 src/orda/main/tools/aim_view.py              # 현재 설정 그대로
    python3 src/orda/main/tools/aim_view.py --ref 0.42   # 후보값을 함께 겹쳐 본다
    python3 src/orda/main/tools/aim_view.py --ref 0.42 0.50   # 여러 개 비교

창에서 s 를 누르면 현재 화면을 PNG 로 저장한다. q 로 종료.
"""

import argparse
import json
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int16, Int32MultiArray

QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)

DEFAULT_CONFIG = ('/home/xytron/xycar_ws/install/lane_detection/share/'
                  'lane_detection/lane_detection_parameter.json')


def build_geometry(config):
    """lane_detection.cpp 와 **같은 정수 절삭**으로 사다리꼴과 호모그래피를 만든다.

    float 로 계산하면 꼭짓점이 1~2px 어긋나 조준선도 그만큼 틀어진다. C++ 이
    int 로 자르는 지점을 그대로 흉내내야 화면에 그린 선이 실제 기준선이다.
    """
    width = int(config['frame_width'])
    height = int(config['frame_height'])
    top_width = int(width * config['roi_top_width_coefficient'])
    bottom_width = int(width * config['roi_bottom_width_coefficient'])
    top_y = int(height * config['roi_top_y_coefficient'])
    bottom_y = int(height * config['roi_bottom_y_coefficient'])
    cx = width // 2

    src = np.float32([
        [cx - top_width // 2, top_y],
        [cx + top_width // 2, top_y],
        [cx + bottom_width // 2, bottom_y],
        [cx - bottom_width // 2, bottom_y],
    ])
    bev_w = bottom_width if bottom_width > 0 else max(top_width, width)
    bev_h = max(1, bottom_y - top_y)
    dst = np.float32([
        [0.0, 0.0], [bev_w - 1.0, 0.0],
        [bev_w - 1.0, bev_h - 1.0], [0.0, bev_h - 1.0],
    ])
    return {
        'width': width, 'height': height, 'cx': cx,
        'top_y': top_y, 'bottom_y': bottom_y,
        'bev_w': bev_w, 'bev_h': bev_h,
        'src': src,
        'h_inv': cv2.getPerspectiveTransform(dst, src),
    }


def bev_column_to_image(geometry, bev_x):
    """BEV 의 세로선 x=bev_x 를 카메라 좌표의 점열로 되돌린다."""
    ys = np.linspace(0.0, geometry['bev_h'] - 1.0, 40)
    points = np.float32([[[bev_x, y]] for y in ys])
    mapped = cv2.perspectiveTransform(points, geometry['h_inv'])
    return [(float(p[0][0]), float(p[0][1])) for p in mapped]


def draw_polyline(canvas, points, color, thickness=2):
    inside = [(int(round(x)), int(round(y))) for x, y in points
              if -10000 < x < 10000]
    for first, second in zip(inside, inside[1:]):
        cv2.line(canvas, first, second, color, thickness, cv2.LINE_AA)


def ratio_for_mode(config, mode):
    """lane_detection.cpp 의 getTargetRefForMode 와 같은 식.

    CENTER 는 독립 키다. 예전처럼 좌/우 평균으로 되돌리면, 두 값이 0.5 대칭이
    아닐 때(0.63/0.35 → 0.49) 화면에 그린 선이 실제 기준선과 어긋난다.
    """
    if mode == 'CENTER':
        return float(config.get('center_reference_center', 0.5))
    return float(config['center_reference_lane_one'] if mode == 'LANE_ONE'
                 else config['center_reference_lane_two'])


def config_start_mode(config):
    """JSON 의 lane_mode. 첫 /mode_info 가 오기 전까지만 쓰인다."""
    text = str(config.get('lane_mode', 'LANE_TWO')).upper()
    if 'CENTER' in text:
        return 'CENTER'
    return 'LANE_ONE' if 'ONE' in text else 'LANE_TWO'


class AimView(Node):
    def __init__(self, geometry, config, extra_ratios):
        super().__init__('aim_view')
        self.geometry = geometry
        self.config = config
        self.extra_ratios = extra_ratios
        # JSON 의 lane_mode 는 시작값일 뿐이다. main_node 가 /internal/lane_command
        # 로 [legacy_mode, lane] 을 보내면 lane_detection 이 lane_mode_ 를 갈아
        # 끼운다. 그 규칙을 그대로 흉내내야 화면의 조준선이 실제 기준선이다.
        self.mode_name = config_start_mode(config)
        self.prev_mode = None
        self.bridge = CvBridge()
        self.frame = None
        self.fit = None
        self.offset = None
        self.create_subscription(Image, '/resized_image', self._image, QOS)
        self.create_subscription(
            Float32MultiArray, '/lane_fit', self._fit, QOS)
        self.create_subscription(Int16, '/lane_offset', self._offset, QOS)
        self.create_subscription(
            Int32MultiArray, '/internal/lane_command', self._command, QOS)

    def _command(self, msg):
        if len(msg.data) < 2:
            return
        new_mode, new_lane = int(msg.data[0]), int(msg.data[1])
        # lane_detection.cpp modeCallback 과 같은 갱신 조건.
        if 0 <= new_lane <= 2 and (
                new_mode == 5
                or (self.prev_mode != 3 and new_mode == 3)):
            self.mode_name = ('LANE_ONE' if new_lane == 1
                              else 'LANE_TWO' if new_lane == 2 else 'CENTER')
        self.prev_mode = new_mode

    @property
    def base_ratio(self):
        return ratio_for_mode(self.config, self.mode_name)

    def _image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _fit(self, msg):
        if len(msg.data) >= 2:
            self.fit = (float(msg.data[0]), float(msg.data[1]))

    def _offset(self, msg):
        self.offset = int(msg.data)

    def render(self):
        if self.frame is None:
            return None
        geometry = self.geometry
        canvas = self.frame.copy()

        # ROI 사다리꼴 — 이 안에서만 차선을 본다.
        cv2.polylines(canvas, [geometry['src'].astype(np.int32)],
                      True, (0, 200, 200), 1, cv2.LINE_AA)
        # 카메라 중심 (비교 기준)
        cv2.line(canvas, (geometry['cx'], geometry['top_y']),
                 (geometry['cx'], geometry['height'] - 1), (130, 130, 130), 1)

        # 조준선: "중앙선이 여기 보이도록" 조향한다.
        rows = []
        base = self.base_ratio
        for ratio, color, label in (
                [(base, (0, 255, 0), f'{self.mode_name} {base:.3f}')]
                + [(r, (255, 160, 0), f'후보 {r:.2f}') for r in self.extra_ratios]):
            bev_x = ratio * geometry['bev_w']
            points = bev_column_to_image(geometry, bev_x)
            draw_polyline(canvas, points, color, 2 if ratio == base else 1)
            bottom = points[-1][0]
            rows.append((label, color, bev_x, bottom))

        # 실제로 검출된 중앙선 (프레임 좌표 x = m*y + b)
        if self.fit is not None:
            slope, intercept = self.fit
            y0, y1 = geometry['top_y'], geometry['height'] - 1
            draw_polyline(
                canvas,
                [(slope * y0 + intercept, y0), (slope * y1 + intercept, y1)],
                (0, 0, 255), 2)

        panel = np.zeros((22 * (len(rows) + 3), canvas.shape[1], 3), np.uint8)
        def put(index, text, color=(230, 230, 230)):
            cv2.putText(panel, text, (8, 16 + 22 * index),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        put(0, f'BEV {geometry["bev_w"]}x{geometry["bev_h"]}  '
               f'camera centre x={geometry["cx"]}')
        for index, (label, color, bev_x, bottom) in enumerate(rows):
            put(index + 1,
                f'{label}: BEV x={bev_x:.0f}  camera x={bottom:.0f} '
                f'({bottom - geometry["cx"]:+.0f} from centre)', color)
        put(len(rows) + 1,
            f'/lane_offset = {self.offset if self.offset is not None else "-"}'
            f'   (red = detected centre line)', (0, 0, 255))
        return np.vstack([canvas, panel])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--ref', type=float, nargs='*', default=[],
                        help='함께 겹쳐 볼 후보 기준 비율')
    parser.add_argument('--out', default='/tmp/aim_view.png')
    args = parser.parse_args()

    with open(args.config) as handle:
        config = json.load(handle)
    geometry = build_geometry(config)
    print(f'설정 파일 : {args.config}')
    print(f'JSON lane_mode: {config_start_mode(config)} (시작값)')
    for mode in ('CENTER', 'LANE_ONE', 'LANE_TWO'):
        ratio = ratio_for_mode(config, mode)
        bottom = bev_column_to_image(geometry, ratio * geometry['bev_w'])[-1][0]
        print(f'  {mode:<9} ratio {ratio:.3f}  카메라 하단 x={bottom:.0f} '
              f'({bottom - geometry["cx"]:+.0f} from centre)')
    print('실제 모드는 /internal/lane_command 를 받아 창에 표시된다.')
    print('\n창: s 저장, q 종료')

    rclpy.init()
    node = AimView(geometry, config, args.ref)
    window = 'aim view  (green=aim, red=detected, yellow=ROI)'
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            view = node.render()
            if view is None:
                continue
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                cv2.imwrite(args.out, view)
                print(f'저장: {args.out}')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
