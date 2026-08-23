"""ROS 2 inference node for the official PIDNet-S lane model."""
from pathlib import Path
import time
import cv2
import numpy as np
import torch
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from .pidnet import PIDNet
from .core import (
    CENTER_LANE_MIN_SUPPORT_RATIO,
    CENTER_LANE_SUPPORT_RADIUS,
    RAIL_MIN_SUPPORT_RATIO,
    RAIL_SUPPORT_RADIUS,
    ROAD_CLASS,
    SHORTCUT_CLASS,
    filter_center_lane_off_surface,
    filter_rails_off_surface,
    label_roi_top,
)

CLASS_NAMES = ('background', 'center_lane', 'left_solid', 'right_solid', 'road', 'shortcut')
# /mode_info 계약(main 의 mode_info.ExternalModeInfoCode)에서 주행면이 정해지는 두 모드.
# lane_drive 면 road 위 중앙선이, shortcut 이면 shortcut 위 중앙선이 우선이다.
# 나머지 모드(대기/라바콘/회피/추월)는 우선면 없이 road·shortcut 을 모두 받는다.
PREFERRED_SURFACE_BY_MODE = {1: ROAD_CLASS, 5: SHORTCUT_CLASS}
CLASS_COLORS = np.array([
    (0, 0, 0),       # background: black
    (0, 255, 255),   # center_lane: yellow
    (255, 80, 0),    # left_solid: blue
    (0, 80, 255),    # right_solid: red
    (90, 90, 90),    # road: gray
    (255, 0, 255),   # shortcut: magenta
], dtype=np.uint8)


def latest_frame_qos():
    """Keep only the newest image while preserving production QoS semantics."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def resize_for_model(bgr, width, height):
    """Skip only the redundant exact-size resize."""
    if bgr.shape[1] == width and bgr.shape[0] == height:
        return bgr
    return cv2.resize(bgr, (width, height), interpolation=cv2.INTER_LINEAR)


def warmup_runner(runner, enabled=True):
    """Run exactly one startup inference, returning its elapsed milliseconds."""
    if not enabled:
        return None
    started = time.perf_counter()
    runner.predict(np.zeros((runner.height, runner.width, 3), dtype=np.uint8))
    if runner.device.type == 'cuda':
        torch.cuda.synchronize(runner.device)
    return (time.perf_counter() - started) * 1000.0


def make_class_visualization(frame, labels):
    colors = CLASS_COLORS[np.clip(labels, 0, len(CLASS_COLORS) - 1)]
    overlay = cv2.addWeighted(frame, 0.55, colors, 0.45, 0.0)
    color_panel = colors.copy()
    overlay_panel = overlay.copy()
    cv2.putText(color_panel, 'PIDNet-S classes', (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    cv2.putText(overlay_panel, 'Overlay', (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    canvas = cv2.hconcat((color_panel, overlay_panel))
    legend = np.zeros((58, canvas.shape[1], 3), dtype=np.uint8)
    x = 12
    for class_id, name in enumerate(CLASS_NAMES):
        color = tuple(int(value) for value in CLASS_COLORS[class_id])
        cv2.rectangle(legend, (x, 13), (x + 24, 37), color, -1)
        text = f'{class_id}:{name}'
        cv2.putText(legend, text, (x + 30, 33),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, (240, 240, 240), 1)
        x += 48 + cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, .48, 1)[0][0]
    return colors, overlay, cv2.vconcat((canvas, legend))


class PIDNetRunner:
    """PIDNet-S 추론기.

    Orin 에서 이 모델은 연산량이 아니라 **커널 런치 오버헤드**에 묶여 있다.
    입력을 640x360 에서 448x256 으로 줄여도 29.5ms -> 29.8ms 로 전혀 빨라지지
    않는 것이 그 증거다. 그래서 아래 최적화는 "계산을 줄이는" 방향이 아니라
    "런치 횟수를 줄이는" 방향이다.

    실측(Jetson Orin NX Super, 40W, 640x360, predict() 전체):
        기존 (fp32 autocast + augment=True)        42.0 ms   (23.8 Hz)
        + augment=False (학습용 보조 헤드 제거)     40.6 ms   (24.6 Hz)
        + GPU 전처리                               33.4 ms   (30.0 Hz)
        + channels_last                            30.2 ms   (33.1 Hz)
        + fp16 가중치                              28.4 ms   (35.2 Hz)
        + CUDA Graph                                9.2 ms  (108.7 Hz)   <-- 현재
    출력 라벨은 기존 경로와 동일하다(그래프 안에서도 logits 를 640x360 으로
    bilinear 업샘플한 뒤 argmax 한다). 1/8 해상도에서 argmax 하면 7.0ms 로 더
    빨라지지만 라벨 경계가 8px 블록으로 뭉개져 차선 피팅이 흔들리므로 쓰지
    않는다(7.7ms 와의 차이가 0.7ms 뿐이라 얻는 것도 없다).
    """

    def __init__(self, model_path, device='auto', width=640, height=360,
                 center_lane_support_radius=CENTER_LANE_SUPPORT_RADIUS,
                 center_lane_min_support_ratio=CENTER_LANE_MIN_SUPPORT_RATIO,
                 rail_support_radius=RAIL_SUPPORT_RADIUS,
                 rail_min_support_ratio=RAIL_MIN_SUPPORT_RATIO,
                 use_cuda_graph=True):
        path=Path(model_path).expanduser().resolve()
        if not path.is_file(): raise FileNotFoundError(f'PIDNet checkpoint not found: {path}')
        name=('cuda' if torch.cuda.is_available() else 'cpu') if device=='auto' else device
        self.device=torch.device(name); self.width=int(width); self.height=int(height)
        self.center_lane_support_radius=int(center_lane_support_radius)
        self.center_lane_min_support_ratio=float(center_lane_min_support_ratio)
        self.rail_support_radius=int(rail_support_radius)
        self.rail_min_support_ratio=float(rail_min_support_ratio)
        self.preferred_surface=None
        checkpoint=torch.load(path,map_location=self.device,weights_only=False); config=checkpoint.get('config',{}); classes=int(config.get('classes',6))
        # 체크포인트에는 보조 헤드(seghead_p/seghead_d) 가중치가 들어 있으므로
        # strict=True 로드를 위해 augment=True 로 만든 뒤, 로드가 끝나고 나서
        # 끈다. 보조 헤드는 학습 손실용이라 추론 결과에 쓰이지 않는다.
        self.model=PIDNet(m=2,n=3,num_classes=classes,planes=32,ppm_planes=96,head_planes=128,augment=True).to(self.device)
        self.model.load_state_dict(checkpoint['model'],strict=True); self.model.eval()
        self.model.augment=False
        self.mean=np.array([.485,.456,.406],np.float32); self.std=np.array([.229,.224,.225],np.float32)
        self.checkpoint=path; self.best_epoch=checkpoint.get('epoch'); self.classes=classes
        self.last_center_lane_removed_px=0
        self.last_rail_removed_px=0
        self.graph=None
        if self.device.type=='cuda':
            torch.backends.cudnn.benchmark=True
            self.model=self.model.half().to(memory_format=torch.channels_last)
            self._mean_gpu=torch.from_numpy(self.mean).to(self.device).half().view(1,3,1,1)
            self._std_gpu=torch.from_numpy(self.std).to(self.device).half().view(1,3,1,1)
            if use_cuda_graph: self._capture_graph()

    def _forward(self,tensor):
        """logits -> 입력 해상도 라벨. 그래프 캡처와 eager 경로가 함께 쓴다."""
        output=self.model(tensor); logits=output[1] if isinstance(output,(list,tuple)) else output
        logits=torch.nn.functional.interpolate(logits,(self.height,self.width),mode='bilinear',align_corners=False)
        return logits.argmax(1)[0].byte()

    def _capture_graph(self):
        """고정 크기 입력으로 CUDA Graph 를 캡처한다. 실패하면 eager 로 남는다."""
        self._static_in=torch.zeros(
            1,3,self.height,self.width,device=self.device,dtype=torch.half,
        ).contiguous(memory_format=torch.channels_last)
        try:
            # 캡처 전에 보조 스트림에서 워밍업해야 cudnn 알고리즘 선택과 지연
            # 초기화가 그래프 안으로 딸려 들어가지 않는다.
            side=torch.cuda.Stream(); side.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(side), torch.inference_mode():
                for _ in range(5): self._forward(self._static_in)
            torch.cuda.current_stream().wait_stream(side)
            graph=torch.cuda.CUDAGraph()
            with torch.inference_mode(), torch.cuda.graph(graph):
                self._static_out=self._forward(self._static_in)
            self.graph=graph
        except Exception:
            # 그래프를 못 쓰면 성능만 잃고 동작은 같다. 주행을 막을 이유가 없다.
            self.graph=None

    def _preprocess(self,bgr):
        resized=resize_for_model(bgr,self.width,self.height)
        rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB)
        tensor=torch.from_numpy(rgb).to(self.device,non_blocking=True)
        tensor=tensor.permute(2,0,1).unsqueeze(0).half().div_(255.)
        return ((tensor-self._mean_gpu)/self._std_gpu).contiguous(memory_format=torch.channels_last)

    def predict(self,bgr):
        original_size=(bgr.shape[1],bgr.shape[0])
        if self.device.type=='cuda':
            tensor=self._preprocess(bgr)
            if self.graph is not None:
                self._static_in.copy_(tensor); self.graph.replay()
                labels=self._static_out.cpu().numpy()
            else:
                with torch.inference_mode(): labels=self._forward(tensor).cpu().numpy()
        else:
            resized=resize_for_model(bgr,self.width,self.height)
            rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)/255.; rgb=(rgb-self.mean)/self.std
            tensor=torch.from_numpy(rgb.transpose(2,0,1).copy()).unsqueeze(0).to(self.device)
            with torch.inference_mode(): labels=self._forward(tensor).cpu().numpy()
        if (self.width,self.height)!=original_size: labels=cv2.resize(labels,original_size,interpolation=cv2.INTER_NEAREST)
        # 트랙 밖 오검출 제거: 주행면(road/shortcut) 위에 놓인 center_lane 만 남긴다.
        labels,self.last_center_lane_removed_px=filter_center_lane_off_surface(
            labels,
            radius=self.center_lane_support_radius,
            min_support_ratio=self.center_lane_min_support_ratio,
            preferred_surface=self.preferred_surface,
        )
        # 바깥 실선도 같은 근거로 거른다. 가드레일이 이 두 클래스를 필터 없이
        # 읽어 좌·우 여유를 min 으로 합치기 때문에, 트랙 밖 오검출 하나가
        # 반대쪽 여유까지 줄여 조향을 엉뚱하게 민다. 임계값은 중앙선보다 훨씬
        # 낮다 — 실선은 바깥쪽이 정당하게 background 다(core.py 주석 참고).
        labels,self.last_rail_removed_px=filter_rails_off_surface(
            labels,
            radius=self.rail_support_radius,
            min_support_ratio=self.rail_min_support_ratio,
        )
        return labels


class PIDNetInferenceNode(Node):
    def __init__(self):
        super().__init__('pidnet_inference_node')
        default_model = str(
            Path(get_package_share_directory('segmentation_tools'))
            / 'model' / 'pidnet_s_best.pt'
        )
        model_path=self.declare_parameter('model_path',default_model).value
        device=self.declare_parameter('device','auto').value
        input_topic=self.declare_parameter('input_topic','/resized_image').value
        class_topic=self.declare_parameter('class_topic','/pidnet_class_map').value
        mode_topic=self.declare_parameter('mode_topic','/mode_info').value
        color_topic=self.declare_parameter('color_topic','/pidnet_color_map').value
        overlay_topic=self.declare_parameter('overlay_topic','/pidnet_overlay').value
        self.show_visualization=bool(self.declare_parameter('show_visualization',False).value)
        self.roi_crop_visualization=bool(self.declare_parameter('roi_crop_visualization',True).value)
        width=self.declare_parameter('model_width',640).value; height=self.declare_parameter('model_height',360).value
        # 중앙선은 주행면 위에만 존재한다. 트랙 밖 오검출을 지우는 검증 파라미터로,
        # radius<=0 또는 ratio<=0 이면 검증을 끄고 모델 출력을 그대로 내보낸다.
        support_radius=self.declare_parameter(
            'center_lane_support_radius',CENTER_LANE_SUPPORT_RADIUS).value
        support_ratio=self.declare_parameter(
            'center_lane_min_support_ratio',CENTER_LANE_MIN_SUPPORT_RATIO).value
        # 바깥 실선도 같은 검증을 받는다. 임계값만 다르다 — 실선은 바깥쪽이
        # 정당하게 background 라 중앙선의 0.5 를 그대로 쓰면 진짜를 지운다.
        # radius<=0 또는 ratio<=0 이면 레일 검증만 꺼진다(중앙선은 그대로).
        rail_radius=self.declare_parameter(
            'rail_support_radius',RAIL_SUPPORT_RADIUS).value
        rail_ratio=self.declare_parameter(
            'rail_min_support_ratio',RAIL_MIN_SUPPORT_RATIO).value
        warmup_on_start=bool(self.declare_parameter('warmup_on_start',True).value)
        self.runner=PIDNetRunner(
            model_path,device,width,height,
            center_lane_support_radius=support_radius,
            center_lane_min_support_ratio=support_ratio,
            rail_support_radius=rail_radius,
            rail_min_support_ratio=rail_ratio)
        if warmup_on_start:
            try:
                elapsed_ms=warmup_runner(self.runner)
            except Exception as error:
                raise RuntimeError(f'PIDNet startup warm-up failed: {error}') from error
            self.get_logger().info(
                f'PIDNet-S ready: model={self.runner.checkpoint} device={self.runner.device} '
                f'epoch={self.runner.best_epoch} warmup_elapsed_ms={elapsed_ms:.1f} ready=true'
            )
        else:
            self.get_logger().info(
                f'PIDNet-S ready: model={self.runner.checkpoint} device={self.runner.device} '
                f'epoch={self.runner.best_epoch} warmup_elapsed_ms=disabled ready=true'
            )
        # GPU 가속 경로가 실제로 활성화됐는지 기동 로그에서 확인한다.
        self.get_logger().info(
            f'PIDNet-S accel: device={self.runner.device} '
            f'cuda_graph={"on" if self.runner.graph is not None else "off"} '
            f'dtype={"fp16" if self.runner.device.type=="cuda" else "fp32"} '
            f'aux_heads={"on" if self.runner.model.augment else "off"}'
        )
        self.bridge=CvBridge(); self.frames=0; self.total_time=0.; self.window_time=0.
        self.center_lane_removed_px=0
        self.rail_removed_px=0
        self.roi_top=label_roi_top(int(height))
        image_qos=latest_frame_qos()
        self.class_pub=self.create_publisher(Image,class_topic,image_qos)
        self.color_pub=self.create_publisher(Image,color_topic,image_qos)
        self.overlay_pub=self.create_publisher(Image,overlay_topic,image_qos)
        self.sub=self.create_subscription(Image,input_topic,self.image_callback,image_qos)
        # 모드에 따라 어느 주행면 위의 중앙선을 우선할지 바뀐다. main 이 mode_info 를
        # 내기 전(부팅 직후)에는 우선면 없이 road/shortcut 을 모두 받는다.
        self.mode_sub=self.create_subscription(Int16,mode_topic,self.mode_callback,image_qos)
        self.window_name='PIDNet-S Class Visualization'
        if self.show_visualization:
            title=f'{self.window_name} (label ROI y>={self.roi_top})' if self.roi_crop_visualization else f'{self.window_name} (full frame)'
            self.window_title=title
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowTitle(self.window_name, title)
        self.get_logger().info(f'PIDNet-S loaded: {self.runner.checkpoint} epoch={self.runner.best_epoch} device={self.runner.device} center_lane_support_radius={self.runner.center_lane_support_radius} center_lane_min_support_ratio={self.runner.center_lane_min_support_ratio} rail_support_radius={self.runner.rail_support_radius} rail_min_support_ratio={self.runner.rail_min_support_ratio} roi_crop_visualization={self.roi_crop_visualization} roi_top={self.roi_top}')

    def mode_callback(self,msg):
        surface=PREFERRED_SURFACE_BY_MODE.get(int(msg.data))
        if surface==self.runner.preferred_surface: return
        self.runner.preferred_surface=surface
        self.get_logger().info(
            f'center_lane preferred surface -> {CLASS_NAMES[surface] if surface else "any drivable"} '
            f'(mode_info={int(msg.data)})')

    def image_callback(self,msg):
        try: frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge input error: {error}'); return
        started=time.perf_counter()
        try: labels=self.runner.predict(frame)
        except Exception as error:
            self.get_logger().error(f'PIDNet inference error: {error}'); return
        class_msg=self.bridge.cv2_to_imgmsg(labels,encoding='mono8'); class_msg.header=msg.header; self.class_pub.publish(class_msg)
        # 색상/오버레이는 사람이 보기 위한 것이다. 아무도 구독하지 않는데도
        # 매 프레임 만들면 6.6ms(전체 49ms 중 13%)를 그냥 버린다. 주행 중에는
        # 보통 구독자가 없으므로 실제로 필요할 때만 계산한다.
        want_color=self.color_pub.get_subscription_count()>0
        want_overlay=self.overlay_pub.get_subscription_count()>0
        # roi_crop_visualization 이면 아래에서 잘라낸 프레임으로 다시 만든다.
        # 그때 전체 프레임 캔버스까지 만들면 같은 일을 두 번 하는 셈이라,
        # 창을 띄운 채 주행할 때 6.4ms 를 그냥 버리게 된다.
        want_full_canvas=self.show_visualization and not self.roi_crop_visualization
        if want_color or want_overlay or want_full_canvas:
            colors,overlay,visualization=make_class_visualization(frame,labels)
            if want_color:
                color_msg=self.bridge.cv2_to_imgmsg(colors,encoding='bgr8'); color_msg.header=msg.header; self.color_pub.publish(color_msg)
            if want_overlay:
                overlay_msg=self.bridge.cv2_to_imgmsg(overlay,encoding='bgr8'); overlay_msg.header=msg.header; self.overlay_pub.publish(overlay_msg)
        if self.show_visualization:
            if self.roi_crop_visualization:
                _,_,window_visualization=make_class_visualization(frame[self.roi_top:],labels[self.roi_top:])
            else:
                window_visualization=visualization
            cv2.imshow(self.window_name,window_visualization)
            cv2.waitKey(1)
        elapsed=time.perf_counter()-started
        self.frames+=1; self.total_time+=elapsed; self.window_time+=elapsed
        self.center_lane_removed_px+=self.runner.last_center_lane_removed_px
        self.rail_removed_px+=self.runner.last_rail_removed_px
        if self.frames%100==0:
            self.get_logger().info(
                f'inference last100={10*self.window_time:.1f} ms '
                f'({100/max(self.window_time,1e-9):.1f} Hz) '
                f'lifetime={1000*self.total_time/self.frames:.1f} ms ({self.frames} frames) '
                f'center_lane_off_surface_removed_px={self.center_lane_removed_px} '
                f'rail_off_surface_removed_px={self.rail_removed_px}'
            )
            self.window_time=0.


    def destroy_node(self):
        if self.show_visualization:
            cv2.destroyWindow('PIDNet-S Class Visualization')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=PIDNetInferenceNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__': main()
