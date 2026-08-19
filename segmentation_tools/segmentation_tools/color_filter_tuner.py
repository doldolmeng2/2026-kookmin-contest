import argparse
import cv2

from .bag_source import BagFrameSource
from .core import (CLASS_NAMES, class_mask, default_config_path, generate_label,
                   load_config, overlay, save_config)


WINDOW = 'Color filter tuner'
CONTROL = 'Playback / class'
SPACE_WINDOWS = {'hls': 'HLS controls', 'hsv': 'HSV controls', 'ycrcb': 'YCrCb controls'}
CHANNELS = (
    ('HLS H min', 'hls', 'min', 0, 179), ('HLS H max', 'hls', 'max', 0, 179),
    ('HLS L min', 'hls', 'min', 1, 255), ('HLS L max', 'hls', 'max', 1, 255),
    ('HLS S min', 'hls', 'min', 2, 255), ('HLS S max', 'hls', 'max', 2, 255),
    ('H min', 'hsv', 'min', 0, 179), ('H max', 'hsv', 'max', 0, 179),
    ('S min', 'hsv', 'min', 1, 255), ('S max', 'hsv', 'max', 1, 255),
    ('V min', 'hsv', 'min', 2, 255), ('V max', 'hsv', 'max', 2, 255),
    ('Y min', 'ycrcb', 'min', 0, 255), ('Y max', 'ycrcb', 'max', 0, 255),
    ('Cr min', 'ycrcb', 'min', 1, 255), ('Cr max', 'ycrcb', 'max', 1, 255),
    ('Cb min', 'ycrcb', 'min', 2, 255), ('Cb max', 'ycrcb', 'max', 2, 255),
)


class Tuner:
    def __init__(self, source, config_path):
        self.source = source
        self.config_path = config_path
        self.config = load_config(config_path)
        self.class_id = 1
        self.index = 0
        self.playing = False
        self.updating = False
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(CONTROL, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROL, 420, 130)
        cv2.moveWindow(CONTROL, 0, 0)
        cv2.createTrackbar('time', CONTROL, 0, max(1, len(source) - 1), self.on_time)
        cv2.createTrackbar('class', CONTROL, 1, 5, self.on_class)
        for column, (space, window) in enumerate(SPACE_WINDOWS.items()):
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 420, 300)
            cv2.moveWindow(window, column * 425, 160)
        for name, space, _, _, maximum in CHANNELS:
            cv2.createTrackbar(name, SPACE_WINDOWS[space], 0, maximum, self.on_filter)
        self.load_sliders()

    def on_time(self, value):
        if not self.updating:
            self.index = min(value, len(self.source) - 1)

    def on_class(self, value):
        if self.updating:
            return
        self.class_id = max(1, value)
        if value == 0:
            cv2.setTrackbarPos('class', CONTROL, 1)
        self.load_sliders()

    def on_filter(self, _value):
        if self.updating:
            return
        current = self.config['classes'][self.class_id]
        for name, space, boundary, channel, _ in CHANNELS:
            current[space][boundary][channel] = cv2.getTrackbarPos(name, SPACE_WINDOWS[space])

    def load_sliders(self):
        self.updating = True
        current = self.config['classes'][self.class_id]
        for name, space, boundary, channel, _ in CHANNELS:
            cv2.setTrackbarPos(name, SPACE_WINDOWS[space], int(current[space][boundary][channel]))
        self.updating = False

    def render(self):
        image = self.source.image(self.index)
        active = class_mask(image, self.config['classes'][self.class_id], self.config.get('combine', 'and'))
        selected = image.copy()
        selected[active == 0] //= 4
        complete = overlay(image, generate_label(image, self.config), self.config, 0.55)
        header = (f'{self.index + 1}/{len(self.source)}  {self.source.seconds(self.index):.2f}s  '
                  f'class {self.class_id}: {CLASS_NAMES[self.class_id]}  combine={self.config.get("combine", "and")}')
        cv2.putText(selected, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, cv2.hconcat([selected, complete]))

    def run(self):
        while True:
            self.render()
            key = cv2.waitKey(30 if self.playing else 1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                save_config(self.config_path, self.config)
                print(f'저장: {self.config_path}')
            elif key == ord(' '):
                self.playing = not self.playing
            elif key in (ord('a'), 81):
                self.index = max(0, self.index - 1)
            elif key in (ord('d'), 83):
                self.index = min(len(self.source) - 1, self.index + 1)
            elif ord('1') <= key <= ord('5'):
                self.class_id = key - ord('0')
                cv2.setTrackbarPos('class', CONTROL, self.class_id)
                self.load_sliders()
            elif key == ord('m'):
                self.config['combine'] = 'or' if self.config.get('combine', 'and') == 'and' else 'and'
            if self.playing:
                self.index = min(len(self.source) - 1, self.index + 1)
                self.playing = self.index < len(self.source) - 1
            cv2.setTrackbarPos('time', CONTROL, self.index)
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description='ROS 2 bag HSV/YCrCb color filter tuner')
    parser.add_argument('bag', help='rosbag2 디렉터리')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--config', default=str(default_config_path()))
    return parser.parse_args()


def main():
    args = parse_args()
    Tuner(BagFrameSource(args.bag, args.topic), args.config).run()


if __name__ == '__main__':
    main()
