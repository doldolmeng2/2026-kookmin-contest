"""Small ROS Image to BGR8 converter used by the ONNX detector."""

from __future__ import annotations

import cv2
import numpy as np


def _rows(msg, bytes_per_pixel: int) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step) if int(msg.step) > 0 else width * bytes_per_pixel
    packed = width * bytes_per_pixel
    if height <= 0 or width <= 0 or step < packed:
        raise ValueError("invalid image dimensions or step")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(f"short image buffer: {raw.size} < {required}")
    return raw[:required].reshape(height, step)[:, :packed]


def imgmsg_to_bgr(msg) -> np.ndarray:
    encoding = str(msg.encoding).strip().lower()
    if encoding in ("bgr8", "rgb8"):
        image = _rows(msg, 3).reshape(msg.height, msg.width, 3)
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)
    if encoding in ("mono8", "8uc1"):
        image = _rows(msg, 1).reshape(msg.height, msg.width)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    if encoding in ("yuv422_yuy2", "yuyv", "yuy2"):
        image = _rows(msg, 2).reshape(msg.height, msg.width, 2)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2))
    if encoding in ("yuv422", "uyvy"):
        image = _rows(msg, 2).reshape(msg.height, msg.width, 2)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY))
    raise ValueError(f"unsupported image encoding: {msg.encoding!r}")
