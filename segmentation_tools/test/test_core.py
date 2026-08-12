import cv2
import numpy as np
import unittest

from segmentation_tools.core import colorize, generate_label


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
        self.assertEqual(tuple(result[0, 0]), (0, 0, 0))
