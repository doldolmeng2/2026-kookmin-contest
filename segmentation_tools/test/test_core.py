import cv2
import numpy as np
import unittest

from segmentation_tools.core import (DEFAULT_COLORS, colorize, draw_roi, generate_label,
                                     overlay)


class CoreTest(unittest.TestCase):
    def test_generate_label_uses_both_color_spaces_and_priority(self):
        image = np.full((3, 4, 3), (0, 255, 255), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[0, 0].tolist()
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)[0, 0].tolist()
        ycc = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)[0, 0].tolist()
        config = {
            'combine': 'and',
            'classes': {
                1: {'priority': 1, 'hls': {'min': hls, 'max': hls}, 'hsv': {'min': hsv, 'max': hsv}, 'ycrcb': {'min': ycc, 'max': ycc}},
                4: {'priority': 5, 'hls': {'min': hls, 'max': hls}, 'hsv': {'min': hsv, 'max': hsv}, 'ycrcb': {'min': ycc, 'max': ycc}},
            },
        }
        label = generate_label(image, config)
        self.assertTrue(np.all(label == 1))

    def test_colorize_keeps_dimensions(self):
        label = np.array([[0, 1, 5]], dtype=np.uint8)
        result = colorize(label)
        self.assertEqual(result.shape, (1, 3, 3))
        self.assertEqual(tuple(result[0, 0]), DEFAULT_COLORS[0])

    def test_overlay_keeps_background_pixels_untouched(self):
        image = np.full((2, 2, 3), 120, dtype=np.uint8)
        label = np.array([[0, 1], [0, 1]], dtype=np.uint8)
        result = overlay(image, label)
        self.assertTrue(np.all(result[:, 0] == 120))
        self.assertFalse(np.all(result[:, 1] == 120))

    def test_draw_roi_dims_outside_and_keeps_inside(self):
        canvas = np.full((360, 640, 3), 200, dtype=np.uint8)
        draw_roi(canvas)
        self.assertTrue(np.all(canvas[100] < 200))
        self.assertTrue(np.all(canvas[340, 200:440] == 200))
