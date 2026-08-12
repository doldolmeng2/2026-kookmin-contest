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
    return cv2.addWeighted(image, 1.0 - alpha, colorize(label, config), alpha, 0)
