import argparse
from pathlib import Path

import cv2
import numpy as np

from .core import (CLASS_NAMES, dataset_items, draw_roi, load_config, overlay,
                   roi_mask, write_preview)


WINDOW = 'Segmentation label editor'
HEADER_HEIGHT = 52
# Diagonal neighbours only, so a flood fill grows one pixel along its staircase
# edges without also widening the flat horizontal and vertical edges.
DIAGONAL_KERNEL = np.uint8([[1, 0, 1], [0, 1, 0], [1, 0, 1]])


def print_controls(dataset, split, sample_count):
    print(f'''\n{'=' * 66}
  세그멘테이션 라벨 검수 도구
  데이터셋 : {dataset}
  분할     : {split} ({sample_count}장)
{'-' * 66}
  마우스 왼쪽 드래그 : 선택한 도구로 계속 칠하기

  b       브러시 모드            g       연결 컴포넌트 모드
  f       색 채우기 모드 (클릭한 곳과 색이 비슷하게 이어진 영역을 칠함)
  [ / ]   브러시 크기 축소/확대  , / .   색 채우기 허용 오차 축소/확대

  0 또는 e  Background           1       중앙차선(노란색)
  2         왼쪽 실선            3       오른쪽 실선
  4         도로                 5       지름길

  z       최근 작업 되돌리기     x       원본/오버레이 전환
  - / +   오버레이 투명도 조절

  a       저장 후 이전           d       저장 후 다음
  c       저장 후 다음           q       저장 후 종료
{'-' * 66}
  흰 선이 lane_detection ROI 경계입니다. 그 바깥은 런타임에서 잘려나가므로
  칠해도 저장되지 않습니다. ROI 안쪽만 라벨링하세요.
{'-' * 66}
  라벨 저장 위치   : labels/{split}/*.png
  Preview 저장 위치: previews/{split}/*.jpg
{'=' * 66}\n''')


class LabelEditor:
    def __init__(self, dataset, split='train', config=None):
        self.root = Path(dataset).expanduser().resolve()
        self.split = split
        self.items = dataset_items(self.root, split)
        self.config = load_config(config) if config else None
        self.index = 0
        self.class_id = 1
        self.mode = 'brush'
        self.brush_size = 11
        # 낮게 시작한다. 이 영상은 바닥 반사와 노란 얼룩이 많아 조금만 키워도 도로로 번진다.
        self.tolerance = 8
        self.alpha = 0.5
        self.drawing = False
        self.button_flag_seen = False
        self.last_point = None
        self.undo_stack = []
        self.show_overlay = True
        self.load()
        print_controls(self.root, self.split, len(self.items))
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, self.mouse)

    def load(self):
        image_path, label_path = self.items[self.index]
        self.image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        self.label = cv2.imread(str(label_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None or self.label is None:
            raise OSError(f'읽기 실패: {image_path} 또는 {label_path}')
        if self.image.shape[:2] != self.label.shape:
            raise ValueError(f'크기 불일치: {image_path}, {label_path}')
        self.roi = roi_mask(self.label.shape)
        self.undo_stack.clear()

    def save(self):
        image_path, label_path = self.items[self.index]
        label_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(label_path), self.label):
            raise OSError(f'저장 실패: {label_path}')
        write_preview(self.root, self.split, image_path.stem, self.image, self.label, self.config)

    def navigate(self, delta, save=True):
        if save:
            self.save()
        self.index = min(max(0, self.index + delta), len(self.items) - 1)
        self.load()

    def push_undo(self):
        self.undo_stack.append(self.label.copy())
        if len(self.undo_stack) > 20:
            self.undo_stack.pop(0)

    def undo(self):
        if self.undo_stack:
            self.label = self.undo_stack.pop()

    def clip_to_roi(self):
        # lane_detection 이 ROI 밖을 잘라내므로 그 바깥은 칠해도 남지 않게 한다.
        self.label[self.roi == 0] = 0

    def apply_fill(self, x, y):
        """클릭한 곳에서 색이 비슷하게 이어진 영역을 현재 클래스로 채운다."""
        height, width = self.label.shape
        mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
        # 라벨링 범위 밖을 미리 막아 두면 채우기가 그 너머로 번지지 않는다.
        mask[1:-1, 1:-1][self.roi == 0] = 1
        difference = (self.tolerance,) * 3
        # FIXED_RANGE 는 이웃이 아니라 클릭한 화소와 색을 비교한다. 이웃과 비교하면
        # 밝기가 완만하게 바뀌는 바닥을 타고 도로 전체로 번진다.
        # MASK_ONLY 라 self.image 는 바뀌지 않고 mask 에만 255 가 찍힌다.
        cv2.floodFill(self.image, mask, (x, y), 0, difference, difference,
                      4 | cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | (255 << 8))
        # 비스듬한 경계는 채우기가 계단처럼 끊겨 한 화소씩 덜 칠해진다.
        # 대각선 방향으로만 1px 넓혀서 그 틈을 메운다.
        region = cv2.dilate(mask, DIAGONAL_KERNEL)[1:-1, 1:-1]
        self.label[(region == 255) & (self.roi > 0)] = self.class_id

    def apply_component(self, x, y):
        old_class = int(self.label[y, x])
        if old_class == self.class_id:
            return
        source = np.uint8(self.label == old_class)
        count, components = cv2.connectedComponents(source, connectivity=8)
        if count > 1:
            component_id = components[y, x]
            self.label[components == component_id] = self.class_id

    def mouse(self, event, x, y, flags, _param):
        # The status header is displayed above the image, not on top of it.
        # Convert window coordinates back to label-image coordinates.
        y -= HEADER_HEIGHT
        if event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.last_point = None
            return
        if event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
            self.button_flag_seen = True
        # Releasing the button outside an OpenCV window does not always send
        # EVENT_LBUTTONUP to this callback, so a stale drawing state can keep
        # painting. The OS button flag catches that, but only on backends that
        # report it on move events at all — checking it unconditionally cancels
        # every drag on the rest. Trust it once this backend has shown it sets it.
        if (event == cv2.EVENT_MOUSEMOVE and self.drawing
                and self.button_flag_seen
                and not (flags & cv2.EVENT_FLAG_LBUTTON)):
            self.drawing = False
            self.last_point = None
            return
        if not (0 <= x < self.label.shape[1] and 0 <= y < self.label.shape[0]):
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.push_undo()
            self.drawing = True
            self.last_point = (x, y)
            if self.mode == 'brush':
                cv2.circle(self.label, (x, y), self.brush_size // 2, self.class_id, -1)
            elif self.mode == 'fill':
                self.apply_fill(x, y)
            else:
                self.apply_component(x, y)
            self.clip_to_roi()
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            if self.mode == 'brush':
                cv2.line(self.label, self.last_point, (x, y), self.class_id, self.brush_size)
            elif self.mode == 'fill':
                self.apply_fill(x, y)
            else:
                self.apply_component(x, y)
            self.last_point = (x, y)
            self.clip_to_roi()
    def render(self):
        canvas = overlay(self.image, self.label, self.config, self.alpha) if self.show_overlay else self.image.copy()
        draw_roi(canvas)
        image_name = self.items[self.index][0].name
        line1 = f'{self.index + 1}/{len(self.items)} {image_name}'
        setting = f'tol={self.tolerance}' if self.mode == 'fill' else f'brush={self.brush_size}'
        line2 = f'{self.mode}  class {self.class_id}:{CLASS_NAMES[self.class_id]}  {setting}'
        header = np.zeros((HEADER_HEIGHT, canvas.shape[1], 3), dtype=np.uint8)
        cv2.putText(header, line1, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(header, line2, (8, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, cv2.vconcat([header, canvas]))

    def run(self):
        while True:
            self.render()
            key = cv2.waitKey(16) & 0xFF
            if key == ord('q'):
                self.save()
                break
            if key == ord('a'):
                self.navigate(-1)
            elif key == ord('d'):
                self.navigate(1)
            elif key == ord('c'):
                self.navigate(1)
            elif key == ord('b'):
                self.mode = 'brush'
            elif key == ord('g'):
                self.mode = 'component'
            elif key == ord('f'):
                self.mode = 'fill'
            elif key == ord('['):
                self.brush_size = max(1, self.brush_size - 2)
            elif key == ord(']'):
                self.brush_size = min(201, self.brush_size + 2)
            elif key == ord(','):
                self.tolerance = max(1, self.tolerance - 2)
            elif key == ord('.'):
                self.tolerance = min(120, self.tolerance + 2)
            elif key in (ord('0'), ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
                self.class_id = key - ord('0')
            elif key == ord('e'):
                self.class_id = 0
            elif key in (ord('z'), ord('Z')):
                self.undo()
            elif key == ord('x'):
                self.show_overlay = not self.show_overlay
            elif key == ord('-'):
                self.alpha = max(0.1, self.alpha - 0.1)
            elif key in (ord('+'), ord('=')):
                self.alpha = min(0.9, self.alpha + 0.1)
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description='픽셀 단위 semantic label 검수 도구')
    parser.add_argument('dataset', help='extract_dataset 출력 디렉터리')
    parser.add_argument('--split', choices=('train', 'val'), default='train')
    parser.add_argument('--config', help='색상 표시용 color_filters.yaml')
    return parser.parse_args()


def main():
    args = parse_args()
    LabelEditor(args.dataset, args.split, args.config).run()


if __name__ == '__main__':
    main()
