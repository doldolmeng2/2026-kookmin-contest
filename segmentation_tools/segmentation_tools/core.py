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
# 중앙선 검증에 쓰는 클래스 번호. CLASS_NAMES 와 같은 값을 이름으로 고정해 둔다.
BACKGROUND_CLASS = 0
CENTER_LANE_CLASS = 1
# 중앙선이 그 위에 그려져 있어야 하는 주행면. road 와 shortcut 둘 뿐이다.
ROAD_CLASS = 4
SHORTCUT_CLASS = 5
DRIVABLE_SURFACE_CLASSES = (ROAD_CLASS, SHORTCUT_CLASS)
# center_lane 성분 주변 몇 화소까지를 "붙어 있다" 로 볼지. 640x360 기준값이다.
CENTER_LANE_SUPPORT_RADIUS = 7
# 테두리의 이 비율 이상이 주행면이어야 진짜 중앙선으로 인정한다.
CENTER_LANE_MIN_SUPPORT_RATIO = 0.5

# 바깥 실선(가드레일이 읽는 클래스). 중앙선과 같은 검증을 받아야 하지만
# 임계값은 같을 수 없다 — 아래 RAIL_MIN_SUPPORT_RATIO 주석 참고.
LEFT_SOLID_CLASS = 2
RIGHT_SOLID_CLASS = 3
RAIL_CLASSES = (LEFT_SOLID_CLASS, RIGHT_SOLID_CLASS)
RAIL_SUPPORT_RADIUS = 7
# 중앙선은 양옆이 모두 주행면이라 테두리의 대부분이 road 다(그래서 0.5).
# 바깥 실선은 정의상 한쪽이 트랙 밖이라 테두리의 절반은 정당하게 background 고,
# 화면 가장자리에 걸리거나 코너에서 비스듬히 보이면 더 낮아진다. 0.5 를 그대로
# 쓰면 진짜 실선을 지운다.
#
# 다행히 진짜와 가짜의 간격이 넓다:
#   진짜 실선  : 안쪽이 road  → 대략 0.3 ~ 0.6
#   트랙 밖 오검출 : 사방이 background → 거의 0
# 그래서 "road 가 조금이라도 붙어 있으면 인정" 쪽으로 낮게 잡는다. 0 이 아니라
# 비율로 두는 이유는 road 화소 몇 개가 튀어 들어온 것에 속지 않기 위해서다.
RAIL_MIN_SUPPORT_RATIO = 0.15

# ROI 밖을 덮는 암전 세기. 라벨 대상이 아닌 영역을 한눈에 구분하기 위한 값이다.
ROI_DIM_STRENGTH = 0.72

# 라벨링 대상은 프레임 아래 40% 뿐이다. 그 위는 칠해도 저장하지 않는다.
# lane_detection 런타임 ROI도 같은 y=216에서 시작해 학습된 전방
# 범위를 모두 쓰되, 라벨이 없는 위쪽은 절대 참조하지 않는다.
LABEL_ROI_BOTTOM_FRACTION = 0.4

# lane_detection 런타임이 BEV 변환 전에 잘라내는 사다리꼴 ROI.
# lane_detection_parameter.json 의 roi_* 계수와 같은 값이며, lane_detection.cpp 와
# 같은 float 곱 뒤 int 절삭으로 꼭짓점을 구한다
# (640x360에서 윗변 y=216, x=151..489).
LANE_DETECTION_ROI_TOP_Y_COEFFICIENT = 0.6
LANE_DETECTION_ROI_BOTTOM_Y_COEFFICIENT = 1.0
LANE_DETECTION_ROI_TOP_WIDTH_COEFFICIENT = 0.53
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


def drivable_surface_mask(label, surface_classes=DRIVABLE_SURFACE_CLASSES):
    """road/shortcut 화소가 True 인 마스크. np.isin 보다 두 배 이상 빠르다."""
    mask = label == surface_classes[0]
    for class_id in surface_classes[1:]:
        mask |= label == class_id
    return mask


def center_lane_ring(blob, ring_kernel):
    """center_lane 성분을 radius 만큼 부풀린 테두리(성분 자신은 뺀 고리)."""
    return cv2.dilate(blob, ring_kernel).astype(bool) & ~blob.astype(bool)


def support_ratio(ring, surface):
    """테두리 중 주행면이 차지하는 비율.

    테두리가 비면(창이 성분으로 꽉 찬 경우) 판정할 근거가 없으므로 1.0 으로 본다.
    """
    ring_pixels = int(np.count_nonzero(ring))
    if ring_pixels == 0:
        return 1.0
    return float(np.count_nonzero(ring & surface)) / ring_pixels


def lane_components(label, lane_class, radius, surface_classes, preferred_surface):
    """lane_class 연결 성분마다 (창, 성분마스크, 면적, 전체비율, 우선비율) 을 낸다.

    우선비율은 preferred_surface 한 클래스만 센 값이고, preferred_surface 가 None
    이면 전체비율과 같다.
    """
    center = (label == lane_class).astype(np.uint8)
    if not center.any():
        return []
    ring_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    count, components, stats, _ = cv2.connectedComponentsWithStats(center, connectivity=8)
    height, width = label.shape
    found = []
    for index in range(1, count):
        x, y, w, h, area = stats[index]
        # 성분마다 전체 프레임을 훑으면 느리다. 테두리가 들어갈 만큼만 잘라 쓴다.
        pad = radius + 1
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(width, x + w + pad), min(height, y + h + pad)
        window = label[y0:y1, x0:x1]
        blob = (components[y0:y1, x0:x1] == index).astype(np.uint8)
        ring = center_lane_ring(blob, ring_kernel)
        ratio_any = support_ratio(ring, drivable_surface_mask(window, surface_classes))
        ratio_preferred = ratio_any if preferred_surface is None else \
            support_ratio(ring, window == preferred_surface)
        found.append(((y0, y1, x0, x1), blob.astype(bool), int(area),
                      ratio_any, ratio_preferred))
    return found


def center_lane_components(label, radius, surface_classes, preferred_surface):
    """lane_components 의 center_lane 고정판 (기존 호출부 호환용)."""
    return lane_components(
        label, CENTER_LANE_CLASS, radius, surface_classes, preferred_surface)


def filter_center_lane_off_surface(
    label,
    radius=CENTER_LANE_SUPPORT_RADIUS,
    min_support_ratio=CENTER_LANE_MIN_SUPPORT_RATIO,
    surface_classes=DRIVABLE_SURFACE_CLASSES,
    preferred_surface=None,
):
    """주행면 위에 놓이지 않은 center_lane 을 background 로 지운다.

    중앙선은 언제나 주행면 위에 그려져 있다. 트랙 바깥의 노란 물체를 모델이
    center_lane 으로 착각하면 그 성분 주변은 주행면이 아니라 background 다.
    center_lane 연결 성분마다 radius 만큼 부풀린 테두리를 보고, 그 중 주행면 비율이
    min_support_ratio 미만이면 진짜 중앙선이 아니라고 보고 지운다.

    preferred_surface 를 주면(lane_drive 는 road, shortcut 모드는 shortcut) 그 면
    위의 중앙선이 우선이다. 우선 면에 놓인 성분이 하나라도 있으면 그것만 남기고
    나머지는 지운다. 하나도 없을 때만 road/shortcut 을 가리지 않고 판정해, 모드 전환
    구간에서 중앙선을 통째로 잃지 않게 한다.

    (지워진 라벨, 지운 화소 수) 를 돌려준다. 지울 것이 없으면 입력 배열을 그대로
    돌려주므로 정상 프레임에서는 복사 비용이 들지 않는다.
    """
    if label.ndim != 2:
        raise ValueError('label must be a 2D class map')
    radius = int(radius)
    min_support_ratio = float(min_support_ratio)
    if radius <= 0 or min_support_ratio <= 0.0:
        return label, 0
    return filter_lane_off_surface(
        label,
        CENTER_LANE_CLASS,
        radius=radius,
        min_support_ratio=min_support_ratio,
        surface_classes=surface_classes,
        preferred_surface=preferred_surface,
        winner_take_all=True,
    )


def filter_lane_off_surface(
    label,
    lane_class,
    radius,
    min_support_ratio,
    surface_classes=DRIVABLE_SURFACE_CLASSES,
    preferred_surface=None,
    winner_take_all=True,
):
    """주행면에 붙어 있지 않은 lane_class 성분을 background 로 지운다.

    winner_take_all 은 중앙선 전용 규칙이다. 중앙선은 프레임에 하나뿐이라
    "우선 면(road 또는 shortcut) 위의 것이 하나라도 있으면 그것만 남긴다" 가
    모드 전환 구간에서 옳다. 바깥 실선은 좌우가 각각 독립으로 존재해야 하므로
    이 규칙을 쓰면 한쪽이 다른 쪽을 지워 버린다 — 레일은 False 로 부른다.
    """
    if label.ndim != 2:
        raise ValueError('label must be a 2D class map')
    radius = int(radius)
    min_support_ratio = float(min_support_ratio)
    if radius <= 0 or min_support_ratio <= 0.0:
        return label, 0
    components = lane_components(
        label, lane_class, radius, surface_classes, preferred_surface)
    if not components:
        return label, 0
    if winner_take_all:
        on_preferred = {
            index for index, component in enumerate(components)
            if component[4] >= min_support_ratio
        }
        keep = on_preferred if on_preferred else {
            index for index, component in enumerate(components)
            if component[3] >= min_support_ratio
        }
    else:
        keep = {
            index for index, component in enumerate(components)
            if component[3] >= min_support_ratio
        }
    filtered = label
    removed = 0
    for index, ((y0, y1, x0, x1), blob, area, _, _) in enumerate(components):
        if index in keep:
            continue
        if filtered is label:
            filtered = label.copy()
        filtered[y0:y1, x0:x1][blob] = BACKGROUND_CLASS
        removed += area
    return filtered, removed


def filter_rails_off_surface(
    label,
    radius=RAIL_SUPPORT_RADIUS,
    min_support_ratio=RAIL_MIN_SUPPORT_RATIO,
    rail_classes=RAIL_CLASSES,
    surface_classes=DRIVABLE_SURFACE_CLASSES,
):
    """주행면에 붙어 있지 않은 left_solid/right_solid 를 background 로 지운다.

    가드레일(lane_guardrail.hpp)은 이 두 클래스를 필터 없이 그대로 읽고,
    mergeRailMargins 가 두 클래스의 좌·우 여유를 각각 min 으로 합친다. 그래서
    차량 왼쪽에 뜬 가짜 right_solid 덩어리 하나가 왼쪽 여유를 줄여 버리고,
    가드레일이 있지도 않은 레일을 피해 반대쪽으로 조향을 더한다. 여기서 지우면
    그 경로가 막힌다.

    preferred_surface 를 쓰지 않는다. 실선은 road 든 shortcut 이든 자기가 접한
    주행면에 붙어 있으면 진짜다 — 어느 면인지는 중앙선 쪽에서 이미 가린다.

    (지워진 라벨, 지운 화소 수) 를 돌려준다.
    """
    filtered = label
    removed = 0
    for lane_class in rail_classes:
        filtered, class_removed = filter_lane_off_surface(
            filtered,
            lane_class,
            radius=radius,
            min_support_ratio=min_support_ratio,
            surface_classes=surface_classes,
            preferred_surface=None,
            winner_take_all=False,
        )
        removed += class_removed
    return filtered, removed
