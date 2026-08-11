import os
import numpy as np
import cv2
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory

NAMES = ['4-traffic', 'green_light', 'left_green_light', 'orange_light', 'red_light']
GREEN, LEFT, ORANGE, RED = 1, 2, 3, 4

def imgmsg_to_bgr(msg):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('bgr8', 'rgb8'):
        arr = buf.reshape(msg.height, msg.width, 3)
        if enc == 'rgb8':
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr)
    if enc == 'mono8':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    arr = buf.reshape(msg.height, msg.width, -1)
    return np.ascontiguousarray(arr[:, :, :3])

class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('nms', 0.45)
        self.declare_parameter('debounce_frames', 3)
        self.declare_parameter('show_debug', False)
        mp = self.get_parameter('model_path').value
        if not mp:
            mp = os.path.join(get_package_share_directory('traffic_light'), 'model', 'best_traffic.onnx')
        self.conf = float(self.get_parameter('conf').value)
        self.nms = float(self.get_parameter('nms').value)
        self.debounce_n = max(1, int(self.get_parameter('debounce_frames').value))
        self.show_debug = bool(self.get_parameter('show_debug').value)
        cam = self.get_parameter('camera_topic').value
        if not os.path.exists(mp):
            self.get_logger().error(f'model not found: {mp}')
            raise FileNotFoundError(mp)
        self.get_logger().info(f'ONNX load: {mp}')
        self.sess = ort.InferenceSession(mp, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Int32, '/traffic_detection', qos)
        self.create_subscription(Image, cam, self.on_image, qos)
        self._cand = 0; self._cnt = 0; self._stable = 0; self._last = None
        self.get_logger().info(f'traffic_node start (cam={cam}, conf={self.conf}, debounce={self.debounce_n})')

    def on_image(self, msg):
        try:
            img = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f'img convert: {e}'); return
        raw, dets = self.infer(img)
        st = self.debounce(raw)
        m = Int32(); m.data = int(st); self.pub.publish(m)
        if st != self._last:
            self.get_logger().info(f'/traffic_detection = {st} (raw={raw})'); self._last = st
        if self.show_debug:
            self.draw(img, dets, raw, st)

    def infer(self, img):
        h, w = img.shape[:2]
        b = cv2.cvtColor(cv2.resize(img, (640, 640)), cv2.COLOR_BGR2RGB)
        b = np.transpose(b.astype(np.float32) / 255.0, (2, 0, 1))[None]
        out = self.sess.run(None, {self.inp: b})[0][0].T
        boxes, scores, clss = [], [], []
        for row in out:
            cs = row[4:]; cid = int(np.argmax(cs)); sc = float(cs[cid])
            if sc < self.conf: continue
            cx, cy, bw, bh = row[:4]
            boxes.append([int((cx-bw/2)*w/640), int((cy-bh/2)*h/640), int(bw*w/640), int(bh*h/640)])
            scores.append(sc); clss.append(cid)
        dets, present = [], set()
        if boxes:
            for i in np.array(cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.nms)).flatten():
                present.add(clss[i]); dets.append((boxes[i], scores[i], clss[i]))
        if LEFT in present: return 3, dets
        if GREEN in present: return 2, dets
        if RED in present or ORANGE in present: return 1, dets
        return 0, dets

    def debounce(self, raw):
        if raw == self._cand: self._cnt += 1
        else: self._cand = raw; self._cnt = 1
        if self._cnt >= self.debounce_n: self._stable = self._cand
        return self._stable

    def draw(self, img, dets, raw, st):
        col = [(200,200,200),(0,220,0),(255,255,0),(0,165,255),(0,0,255)]
        v = img.copy()
        for (x,y,bw,bh),sc,cid in dets:
            cv2.rectangle(v,(x,y),(x+bw,y+bh),col[cid],2)
            cv2.putText(v,f'{NAMES[cid]} {sc:.2f}',(x,max(12,y-4)),cv2.FONT_HERSHEY_SIMPLEX,0.5,col[cid],1,cv2.LINE_AA)
        cv2.putText(v,f'raw={raw} stable={st}',(6,22),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2,cv2.LINE_AA)
        cv2.imshow('traffic_light', v); cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
