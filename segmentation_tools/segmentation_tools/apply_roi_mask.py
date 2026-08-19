"""데이터셋의 라벨에서 lane_detection ROI 바깥을 background 로 지운다.

ROI 밖은 런타임에서 잘려나가므로 학습에 쓰이지 않는다. 자동 라벨이 ROI 위쪽에
남겨 놓은 화소를 정리해, 라벨 작업 범위를 ROI 안으로 못박는 용도다.
"""
import argparse
from pathlib import Path

import cv2
import yaml

from .core import (LABEL_ROI_BOTTOM_FRACTION, clear_outside_roi, dataset_items,
                   default_config_path, label_roi_top, load_config, write_preview)


def apply_roi_mask(dataset, split='train', config=None, previews=True):
    root = Path(dataset).expanduser().resolve()
    items = dataset_items(root, split)
    changed_frames = 0
    changed_pixels = 0
    for image_path, label_path in items:
        label = cv2.imread(str(label_path), cv2.IMREAD_GRAYSCALE)
        if label is None:
            raise OSError(f'읽기 실패: {label_path}')
        changed = clear_outside_roi(label)
        if changed:
            changed_frames += 1
            changed_pixels += changed
            if not cv2.imwrite(str(label_path), label):
                raise OSError(f'저장 실패: {label_path}')
        if previews:
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise OSError(f'읽기 실패: {image_path}')
            write_preview(root, split, image_path.stem, image, label, config)
    record_roi(root, label.shape)
    return len(items), changed_frames, changed_pixels


def record_roi(root, shape):
    """dataset.yaml 에 적용된 라벨링 범위를 기록해 두어 나중에 재현할 수 있게 한다."""
    dataset_path = Path(root) / 'dataset.yaml'
    if not dataset_path.exists():
        return
    metadata = yaml.safe_load(dataset_path.read_text(encoding='utf-8'))
    metadata.pop('roi_polygon', None)  # 사다리꼴 ROI 를 쓰던 시절의 기록
    metadata['label_roi'] = {
        'bottom_fraction': LABEL_ROI_BOTTOM_FRACTION,
        'top_row': int(label_roi_top(shape[0])),
    }
    dataset_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description='ROI 바깥 라벨을 background 로 지운다.')
    parser.add_argument('dataset', help='데이터셋 폴더 경로')
    parser.add_argument('--split', default='train')
    parser.add_argument('--config', default=None, help='오버레이 색 설정 (color_filters.yaml)')
    parser.add_argument('--no-previews', action='store_true', help='previews 재생성 생략')
    arguments = parser.parse_args()

    config_path = arguments.config or default_config_path()
    config = load_config(config_path) if Path(config_path).exists() else None
    total, changed_frames, changed_pixels = apply_roi_mask(
        arguments.dataset, arguments.split, config, not arguments.no_previews)
    print(f'{total}장 중 {changed_frames}장에서 ROI 밖 라벨 {changed_pixels}화소를 지웠습니다.')


if __name__ == '__main__':
    main()
