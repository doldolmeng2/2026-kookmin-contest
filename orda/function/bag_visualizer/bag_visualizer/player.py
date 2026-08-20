import argparse

import cv2

from segmentation_tools.bag_source import BagFrameSource

WINDOW = 'Bag Visualizer'
TRACKBAR = 'time'


def print_controls(bag_path, topic, frame_count):
    print(f'''
{'=' * 60}
Bag Visualizer: {bag_path}
Topic: {topic}  |  Frames: {frame_count}

  space         재생 / 일시정지
  a / ←         한 프레임 뒤로 (일시정지)
  d / →         한 프레임 앞으로 (일시정지)
  time 슬라이더   드래그로 임의 시점 탐색 (영상처럼)
  q / ESC       종료
{'=' * 60}
''')


class BagVisualizer:
    """rosbag2의 이미지 토픽을 실측 시간 간격대로 재생하고,

    상단 슬라이더를 드래그하면 영상 탐색바처럼 임의 시점으로 즉시 이동한다.
    """

    def __init__(self, source, rate):
        self.source = source
        self.rate = max(0.05, rate)
        self.index = 0
        self.playing = True
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.createTrackbar(TRACKBAR, WINDOW, 0, max(1, len(source) - 1), self.on_time)

    def on_time(self, value):
        self.index = min(value, len(self.source) - 1)

    def render(self):
        image = self.source.image(self.index).copy()
        last = len(self.source) - 1
        header = (
            f'{self.index + 1}/{len(self.source)}  '
            f'{self.source.seconds(self.index):.2f}s / {self.source.seconds(last):.2f}s  '
            f'{"PLAY" if self.playing else "PAUSE"}  x{self.rate:g}'
        )
        cv2.putText(image, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, image)

    def next_delay_ms(self):
        # 다음 프레임까지 실측 시간 간격만큼 대기해 실제 재생 속도로 보여준다.
        last = len(self.source) - 1
        if self.index >= last:
            return 30
        dt = self.source.seconds(self.index + 1) - self.source.seconds(self.index)
        return max(1, int(dt * 1000 / self.rate))

    def run(self):
        while True:
            self.render()
            key = cv2.waitKey(self.next_delay_ms() if self.playing else 30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord(' '):
                self.playing = not self.playing
            elif key in (ord('a'), 81):
                self.playing = False
                self.index = max(0, self.index - 1)
            elif key in (ord('d'), 83):
                self.playing = False
                self.index = min(len(self.source) - 1, self.index + 1)
            if self.playing:
                if self.index < len(self.source) - 1:
                    self.index += 1
                else:
                    self.playing = False
            cv2.setTrackbarPos(TRACKBAR, WINDOW, self.index)
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(
        description='rosbag2 이미지 토픽을 영상처럼 재생/드래그 탐색하는 뷰어')
    parser.add_argument('bag', help='rosbag2 디렉터리 경로')
    parser.add_argument('--topic', default='/resized_image', help='표시할 이미지 토픽 (기본: /resized_image)')
    parser.add_argument('--rate', type=float, default=1.0, help='재생 속도 배율 (기본: 1.0)')
    return parser.parse_args()


def main():
    args = parse_args()
    source = BagFrameSource(args.bag, args.topic)
    print_controls(args.bag, args.topic, len(source))
    BagVisualizer(source, args.rate).run()


if __name__ == '__main__':
    main()
