import os
from pathlib import Path

import cv2
import numpy as np
import yaml


CLASS_NAMES = {
    0: 'background', 1: 'center_lane', 2: 'left_solid',
    3: 'right_solid', 4: 'road', 5: 'shortcut',
}
DEFAULT_COLORS = {
    0: (0, 0, 0), 1: (0, 255, 255), 2: (255, 80, 0),
    3: (0, 80, 255), 4: (80, 80, 80), 5: (255, 0, 255),
}
# ROI 밖을 덮는 암전 세기. 라벨 대상이 아닌 영역을 한눈에 구분하기 위한 값이다.
ROI_DIM_STRENGTH = 0.72

# 라벨링 대상은 프레임 아래 40% 뿐이다. 그 위는 칠해도 저장하지 않는다.
# lane_detection 런타임 ROI(아래 사다리꼴, 윗변 y=251)보다 위까지 잡아 두었다.
# 런타임 ROI 를 나중에 올리더라도 라벨을 다시 만들지 않아도 되게 한 여유분이다.
LABEL_ROI_BOTTOM_FRACTION = 0.4

# lane_detection 런타임이 BEV 변환 전에 잘라내는 사다리꼴 ROI.
# lane_detection_parameter.json 의 roi_* 계수와 같은 값이며, lane_detection.cpp 와
# 똑같이 float 곱 뒤 int 절삭으로 꼭짓점을 구한다 (640x360 에서 윗변 y = 251).
LANE_DETECTION_ROI_TOP_Y_COEFFICIENT = 0.7
LANE_DETECTION_ROI_BOTTOM_Y_COEFFICIENT = 1.0
LANE_DETECTION_ROI_TOP_WIDTH_COEFFICIENT = 0.9
LANE_DETECTION_ROI_BOTTOM_WIDTH_COEFFICIENT = 2.0


def label_roi_top(height):
    """라벨링 대상이 시작하는 행.

    360행에서 아래 40% 가 정확히 144행(216~359)이 되도록 반올림한다. int 절삭을 쓰면
    0.4 가 이진수로 조금 작아 145행이 되어 버린다.
    """
    return height - round(height * LABEL_ROI_BOTTOM_FRACTION)


def roi_mask(shape):
    """라벨링 대상(아래 40%)이 255인 마스크."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    mask[label_roi_top(shape[0]):] = 255
    return mask


def lane_detection_roi_polygon(shape):
    """런타임 ROI 사다리꼴 꼭짓점을 좌상단부터 시계방향으로 돌려준다.

    라벨링에는 쓰지 않는다. 라벨링 범위가 런타임 ROI 를 덮는지 확인하는 기준이다.
    """
    height, width = shape[:2]
    center_x = width // 2
    top_y = int(height * LANE_DETECTION_ROI_TOP_Y_COEFFICIENT)
    bottom_y = int(height * LANE_DETECTION_ROI_BOTTOM_Y_COEFFICIENT)
    half_top = int(width * LANE_DETECTION_ROI_TOP_WIDTH_COEFFICIENT) // 2
    half_bottom = int(width * LANE_DETECTION_ROI_BOTTOM_WIDTH_COEFFICIENT) // 2
    return np.array([
        [center_x - half_top, top_y],
        [center_x + half_top, top_y],
        [center_x + half_bottom, bottom_y],
        [center_x - half_bottom, bottom_y],
    ], dtype=np.int32)


def clear_outside_roi(label):
    """라벨링 범위 밖 라벨을 background 로 지운다. 지워진 화소 수를 돌려준다."""
    outside = roi_mask(label.shape) == 0
    changed = int(np.count_nonzero(label[outside]))
    label[outside] = 0
    return changed


def dataset_items(root, split):
    """lists/<split>.lst 를 읽어 (이미지 경로, 라벨 경로) 목록을 돌려준다."""
    root = Path(root).expanduser().resolve()
    list_path = root / 'lists' / f'{split}.lst'
    if not list_path.exists():
        raise FileNotFoundError(f'목록 파일 없음: {list_path}')
    items = []
    for line in list_path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            image_path, label_path = line.split()[:2]
            items.append((root / image_path, root / label_path))
    if not items:
        raise ValueError(f'{list_path}에 샘플이 없습니다.')
    return items


def write_preview(root, split, stem, image, label, config=None):
    """previews/<split>/<stem>.jpg 를 오버레이로 새로 쓴다."""
    preview_path = Path(root) / 'previews' / split / f'{stem}.jpg'
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = preview_path.with_name(f'{preview_path.stem}.tmp.jpg')
    if not cv2.imwrite(str(temporary_path), draw_roi(overlay(image, label, config, 0.5))):
        raise OSError(f'미리보기 저장 실패: {preview_path}')
    # Older datasets may contain PNG/JPEG previews for the same frame.
    # Keep exactly one canonical JPG instead of leaving the old preview beside it.
    for existing in preview_path.parent.glob(f'{stem}.*'):
        if existing != temporary_path and existing.is_file():
            existing.unlink()
    os.replace(temporary_path, preview_path)
    return preview_path


def default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('segmentation_tools')) / 'config' / 'color_filters.yaml'
        if installed.exists():
            return installed
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[1] / 'config' / 'color_filters.yaml'


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    config['classes'] = {int(key): value for key, value in config['classes'].items()}
    return config


def save_config(path, config):
    output = dict(config)
    output['classes'] = {int(key): value for key, value in config['classes'].items()}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)


def class_mask(image, class_config, combine='and'):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ycc = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    hls_range = class_config.get('hls', {'min': [0, 0, 0], 'max': [179, 255, 255]})
    hsv_range, ycc_range = class_config['hsv'], class_config['ycrcb']
    hls_mask = cv2.inRange(hls, np.uint8(hls_range['min']), np.uint8(hls_range['max']))
    hsv_mask = cv2.inRange(hsv, np.uint8(hsv_range['min']), np.uint8(hsv_range['max']))
    ycc_mask = cv2.inRange(ycc, np.uint8(ycc_range['min']), np.uint8(ycc_range['max']))
    if combine == 'or':
        return cv2.bitwise_or(cv2.bitwise_or(hls_mask, hsv_mask), ycc_mask)
    return cv2.bitwise_and(cv2.bitwise_and(hls_mask, hsv_mask), ycc_mask)


def generate_label(image, config):
    label = np.zeros(image.shape[:2], dtype=np.uint8)
    morphology = config.get('morphology', {})
    # Larger priority values are painted first; the smallest number wins overlaps.
    ordered = sorted(config['classes'].items(), key=lambda item: item[1].get('priority', item[0]), reverse=True)
    for class_id, class_config in ordered:
        mask = class_mask(image, class_config, config.get('combine', 'and'))
        for operation in ('open', 'close'):
            size = int(morphology.get(operation, 0))
            if size > 1:
                kernel = np.ones((size, size), np.uint8)
                opcode = cv2.MORPH_OPEN if operation == 'open' else cv2.MORPH_CLOSE
                mask = cv2.morphologyEx(mask, opcode, kernel)
        label[mask > 0] = class_id
    return label


def colorize(label, config=None):
    output = np.zeros((*label.shape, 3), dtype=np.uint8)
    for class_id in CLASS_NAMES:
        color = DEFAULT_COLORS[class_id]
        if config and class_id in config.get('classes', {}):
            color = tuple(config['classes'][class_id].get('color_bgr', color))
        output[label == class_id] = color
    return output


def overlay(image, label, config=None, alpha=0.5):
    blended = cv2.addWeighted(image, 1.0 - alpha, colorize(label, config), alpha, 0)
    # background 까지 덧칠하면 화면 대부분이 한 색으로 덮여 정작 칠할 곳이 안 보인다.
    # 칠해진 클래스만 물들이고 background 는 원본을 그대로 남긴다.
    background = label == 0
    blended[background] = image[background]
    return blended


def draw_roi(canvas):
    """라벨링 범위 밖을 어둡게 덮고 경계선을 그린다. 캔버스를 제자리에서 고치고 돌려준다."""
    top = label_roi_top(canvas.shape[0])
    canvas[:top] = (canvas[:top] * (1.0 - ROI_DIM_STRENGTH)).astype(np.uint8)
    cv2.line(canvas, (0, top), (canvas.shape[1] - 1, top), (255, 255, 255), 1)
    return canvas
