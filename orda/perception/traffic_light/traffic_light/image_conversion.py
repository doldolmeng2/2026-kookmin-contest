"""ROS Image byte-buffer conversion helpers with explicit encoding handling."""

from __future__ import annotations

import cv2
import numpy as np


def _packed_rows(msg, bytes_per_pixel: int) -> np.ndarray:
    """Return packed image rows while respecting ``sensor_msgs/Image.step``."""

    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step) if int(msg.step) > 0 else width * bytes_per_pixel
    packed_width = width * bytes_per_pixel
    if height <= 0 or width <= 0:
        raise ValueError("image width and height must be positive")
    if step < packed_width:
        raise ValueError(
            f"image step {step} is smaller than packed row {packed_width}"
        )

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(
            f"image buffer has {raw.size} bytes, expected at least {required}"
        )
    return raw[:required].reshape(height, step)[:, :packed_width]


def imgmsg_to_bgr(msg) -> np.ndarray:
    """Convert common ROS image encodings to contiguous OpenCV BGR8.

    The J4012 USB camera publishes ``yuv422_yuy2``.  Treating that two-byte
    format as an arbitrary multi-channel image made the traffic detector feed
    a two-channel frame to ``cv2.cvtColor`` and crash.  Unknown encodings are
    rejected instead of being guessed.
    """

    encoding = str(msg.encoding).strip().lower()
    if encoding in ("bgr8", "rgb8"):
        image = _packed_rows(msg, 3).reshape(msg.height, msg.width, 3)
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if encoding in ("bgra8", "rgba8"):
        image = _packed_rows(msg, 4).reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        return np.ascontiguousarray(cv2.cvtColor(image, code))

    if encoding in ("mono8", "8uc1"):
        image = _packed_rows(msg, 1).reshape(msg.height, msg.width)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))

    if encoding in ("yuv422_yuy2", "yuyv", "yuy2"):
        image = _packed_rows(msg, 2).reshape(msg.height, msg.width, 2)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2))

    if encoding in ("yuv422", "uyvy"):
        image = _packed_rows(msg, 2).reshape(msg.height, msg.width, 2)
        return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY))

    raise ValueError(f"unsupported image encoding: {msg.encoding!r}")
