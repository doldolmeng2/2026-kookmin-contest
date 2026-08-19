"""ROS 2 inference node for the official PIDNet-S lane model."""
from pathlib import Path
import time
import cv2
import numpy as np
import torch
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from .pidnet import PIDNet
from .core import label_roi_top

CLASS_NAMES = ('background', 'center_lane', 'left_solid', 'right_solid', 'road', 'shortcut')
CLASS_COLORS = np.array([
    (0, 0, 0),       # background: black
    (0, 255, 255),   # center_lane: yellow
    (255, 80, 0),    # left_solid: blue
    (0, 80, 255),    # right_solid: red
    (90, 90, 90),    # road: gray
    (255, 0, 255),   # shortcut: magenta
], dtype=np.uint8)


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
    def __init__(self, model_path, device='auto', width=640, height=360, lane_classes=(1,)):
        path=Path(model_path).expanduser().resolve()
        if not path.is_file(): raise FileNotFoundError(f'PIDNet checkpoint not found: {path}')
        name=('cuda' if torch.cuda.is_available() else 'cpu') if device=='auto' else device
        self.device=torch.device(name); self.width=int(width); self.height=int(height); self.lane_classes=tuple(int(x) for x in lane_classes)
        checkpoint=torch.load(path,map_location=self.device,weights_only=False); config=checkpoint.get('config',{}); classes=int(config.get('classes',6))
        self.model=PIDNet(m=2,n=3,num_classes=classes,planes=32,ppm_planes=96,head_planes=128,augment=True).to(self.device)
        self.model.load_state_dict(checkpoint['model'],strict=True); self.model.eval()
        self.mean=np.array([.485,.456,.406],np.float32); self.std=np.array([.229,.224,.225],np.float32)
        if self.device.type=='cuda': torch.backends.cudnn.benchmark=True
        self.checkpoint=path; self.best_epoch=checkpoint.get('epoch'); self.classes=classes

    def predict(self,bgr):
        original_size=(bgr.shape[1],bgr.shape[0]); resized=cv2.resize(bgr,(self.width,self.height),interpolation=cv2.INTER_LINEAR)
        rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB).astype(np.float32)/255.; rgb=(rgb-self.mean)/self.std
        tensor=torch.from_numpy(rgb.transpose(2,0,1).copy()).unsqueeze(0).to(self.device)
        with torch.inference_mode(),torch.autocast(device_type=self.device.type,enabled=self.device.type=='cuda'):
            output=self.model(tensor); logits=output[1] if isinstance(output,(list,tuple)) else output
            logits=torch.nn.functional.interpolate(logits,(self.height,self.width),mode='bilinear',align_corners=False)
            labels=logits.argmax(1)[0].byte().cpu().numpy()
        if (self.width,self.height)!=original_size: labels=cv2.resize(labels,original_size,interpolation=cv2.INTER_NEAREST)
        mask=np.isin(labels,self.lane_classes).astype(np.uint8)*255
        return labels,mask


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
        mask_topic=self.declare_parameter('mask_topic','/lane_segmentation_mask').value
        class_topic=self.declare_parameter('class_topic','/pidnet_class_map').value
        color_topic=self.declare_parameter('color_topic','/pidnet_color_map').value
        overlay_topic=self.declare_parameter('overlay_topic','/pidnet_overlay').value
        self.show_visualization=bool(self.declare_parameter('show_visualization',False).value)
        self.roi_crop_visualization=bool(self.declare_parameter('roi_crop_visualization',True).value)
        width=self.declare_parameter('model_width',640).value; height=self.declare_parameter('model_height',360).value
        lane_classes=self.declare_parameter('lane_classes',[1]).value
        self.runner=PIDNetRunner(model_path,device,width,height,lane_classes); self.bridge=CvBridge(); self.frames=0; self.total_time=0.
        self.roi_top=label_roi_top(int(height))
        self.mask_pub=self.create_publisher(Image,mask_topic,qos_profile_sensor_data)
        self.class_pub=self.create_publisher(Image,class_topic,qos_profile_sensor_data)
        self.color_pub=self.create_publisher(Image,color_topic,qos_profile_sensor_data)
        self.overlay_pub=self.create_publisher(Image,overlay_topic,qos_profile_sensor_data)
        self.sub=self.create_subscription(Image,input_topic,self.image_callback,qos_profile_sensor_data)
        self.window_name='PIDNet-S Class Visualization'
        if self.show_visualization:
            title=f'{self.window_name} (label ROI y>={self.roi_top})' if self.roi_crop_visualization else f'{self.window_name} (full frame)'
            self.window_title=title
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowTitle(self.window_name, title)
        self.get_logger().info(f'PIDNet-S loaded: {self.runner.checkpoint} epoch={self.runner.best_epoch} device={self.runner.device} lane_classes={self.runner.lane_classes} roi_crop_visualization={self.roi_crop_visualization} roi_top={self.roi_top}')

    def image_callback(self,msg):
        try: frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'cv_bridge input error: {error}'); return
        started=time.perf_counter()
        try: labels,mask=self.runner.predict(frame)
        except Exception as error:
            self.get_logger().error(f'PIDNet inference error: {error}'); return
        mask_msg=self.bridge.cv2_to_imgmsg(mask,encoding='mono8'); mask_msg.header=msg.header; self.mask_pub.publish(mask_msg)
        class_msg=self.bridge.cv2_to_imgmsg(labels,encoding='mono8'); class_msg.header=msg.header; self.class_pub.publish(class_msg)
        colors,overlay,visualization=make_class_visualization(frame,labels)
        color_msg=self.bridge.cv2_to_imgmsg(colors,encoding='bgr8'); color_msg.header=msg.header; self.color_pub.publish(color_msg)
        overlay_msg=self.bridge.cv2_to_imgmsg(overlay,encoding='bgr8'); overlay_msg.header=msg.header; self.overlay_pub.publish(overlay_msg)
        if self.show_visualization:
            if self.roi_crop_visualization:
                _,_,window_visualization=make_class_visualization(frame[self.roi_top:],labels[self.roi_top:])
            else:
                window_visualization=visualization
            cv2.imshow(self.window_name,window_visualization)
            cv2.waitKey(1)
        self.frames+=1; self.total_time+=time.perf_counter()-started
        if self.frames%100==0: self.get_logger().info(f'inference average={1000*self.total_time/self.frames:.1f} ms ({self.frames} frames)')


    def destroy_node(self):
        if self.show_visualization:
            cv2.destroyWindow('PIDNet-S Class Visualization')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=PIDNetInferenceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__': main()
