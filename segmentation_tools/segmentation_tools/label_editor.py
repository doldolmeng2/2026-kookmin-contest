import argparse
import os
from pathlib import Path

import cv2
import numpy as np

from .core import CLASS_NAMES, load_config, overlay


WINDOW = 'Segmentation label editor'


class LabelEditor:
    def __init__(self, dataset, split='train', config=None):
        self.root = Path(dataset).expanduser().resolve()
        self.split = split
        list_path = self.root / 'lists' / f'{split}.lst'
        if not list_path.exists():
            raise FileNotFoundError(f'목록 파일 없음: {list_path}')
        self.items = []
        for line in list_path.read_text(encoding='utf-8').splitlines():
            if line.strip():
                image_path, label_path = line.split()[:2]
                self.items.append((self.root / image_path, self.root / label_path))
        if not self.items:
            raise ValueError(f'{list_path}에 샘플이 없습니다.')
        self.config = load_config(config) if config else None
        self.index = 0
        self.class_id = 1
        self.mode = 'brush'
        self.brush_size = 11
        self.alpha = 0.5
        self.drawing = False
        self.last_point = None
        self.undo_stack = []
        self.show_overlay = True
        self.load()
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
        self.undo_stack.clear()

    def save(self):
        image_path, label_path = self.items[self.index]
        label_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(label_path), self.label):
            raise OSError(f'저장 실패: {label_path}')
        preview_path = self.root / 'previews' / self.split / f'{image_path.stem}.jpg'
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        preview = overlay(self.image, self.label, self.config, 0.5)
        temporary_path = preview_path.with_name(f'{preview_path.stem}.tmp.jpg')
        if not cv2.imwrite(str(temporary_path), preview):
            raise OSError(f'미리보기 저장 실패: {preview_path}')
        # Older datasets may contain PNG/JPEG previews for the same frame.
        # Keep exactly one canonical JPG instead of leaving the old preview beside it.
        for existing in preview_path.parent.glob(f'{image_path.stem}.*'):
            if existing != temporary_path and existing.is_file():
                existing.unlink()
        os.replace(temporary_path, preview_path)

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

    def apply_component(self, x, y):
        old_class = int(self.label[y, x])
        if old_class == self.class_id:
            return
        source = np.uint8(self.label == old_class)
        count, components = cv2.connectedComponents(source, connectivity=8)
        if count > 1:
            component_id = components[y, x]
            self.label[components == component_id] = self.class_id

    def mouse(self, event, x, y, _flags, _param):
        if not (0 <= x < self.label.shape[1] and 0 <= y < self.label.shape[0]):
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.push_undo()
            self.drawing = True
            self.last_point = (x, y)
            if self.mode == 'brush':
                cv2.circle(self.label, (x, y), self.brush_size // 2, self.class_id, -1)
            else:
                self.apply_component(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            if self.mode == 'brush':
                cv2.line(self.label, self.last_point, (x, y), self.class_id, self.brush_size)
            else:
                self.apply_component(x, y)
            self.last_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.last_point = None

    def render(self):
        canvas = overlay(self.image, self.label, self.config, self.alpha) if self.show_overlay else self.image.copy()
        image_name = self.items[self.index][0].name
        line1 = f'{self.index + 1}/{len(self.items)} {image_name}'
        line2 = f'{self.mode}  class {self.class_id}:{CLASS_NAMES[self.class_id]}  brush={self.brush_size}'
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 52), (0, 0, 0), -1)
        cv2.putText(canvas, line1, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, line2, (8, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, canvas)

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
            elif key == ord('['):
                self.brush_size = max(1, self.brush_size - 2)
            elif key == ord(']'):
                self.brush_size = min(201, self.brush_size + 2)
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
