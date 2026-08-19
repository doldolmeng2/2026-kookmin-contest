"""Random-ish access to image frames in a ROS 2 bag.

Only serialized messages are retained, so compressed image bags stay small in RAM.
"""
from pathlib import Path

import cv2
import numpy as np


class BagFrameSource:
    def __init__(self, bag_path, topic='/image_raw/compressed'):
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise RuntimeError('ROS 2 환경을 source한 터미널에서 실행하세요.') from exc

        self._deserialize = deserialize_message
        self.topic = topic
        uri = str(Path(bag_path).expanduser().resolve())
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if topic not in topics:
            raise ValueError(f'{topic}이 bag에 없습니다. 사용 가능: {", ".join(topics)}')
        self.message_type_name = topics[topic]
        self.message_type = get_message(self.message_type_name)
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        self.frames = []
        while reader.has_next():
            read_topic, data, timestamp = reader.read_next()
            if read_topic == topic:
                self.frames.append((timestamp, bytes(data)))
        if not self.frames:
            raise ValueError(f'{topic}에 이미지가 없습니다.')
        self.timestamps = [item[0] for item in self.frames]
        self.start_ns = self.frames[0][0]
        self.duration_ns = max(1, self.frames[-1][0] - self.start_ns)

    def __len__(self):
        return len(self.frames)

    def seconds(self, index):
        return (self.frames[index][0] - self.start_ns) / 1e9

    def image(self, index):
        message = self._deserialize(self.frames[index][1], self.message_type)
        if self.message_type_name == 'sensor_msgs/msg/CompressedImage':
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        elif self.message_type_name == 'sensor_msgs/msg/Image':
            from cv_bridge import CvBridge
            image = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
        else:
            raise TypeError(f'지원하지 않는 이미지 메시지: {self.message_type_name}')
        if image is None:
            raise ValueError(f'{index}번 프레임 디코딩에 실패했습니다.')
        return image

    def nearest_index(self, seconds):
        target = self.start_ns + int(seconds * 1e9)
        import bisect
        index = bisect.bisect_left(self.timestamps, target)
        if index == len(self.timestamps):
            return index - 1
        if index and target - self.timestamps[index - 1] < self.timestamps[index] - target:
            return index - 1
        return index
