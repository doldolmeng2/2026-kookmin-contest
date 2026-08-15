import tempfile
import unittest
from pathlib import Path
from contextlib import redirect_stdout
from io import StringIO

import cv2
import numpy as np

from segmentation_tools.core import (clear_outside_roi, label_roi_top,
                                     lane_detection_roi_polygon, roi_mask)
from segmentation_tools.label_editor import HEADER_HEIGHT, LabelEditor, print_controls


class LabelEditorTest(unittest.TestCase):
    def test_mouse_header_is_not_painted_and_image_y_is_adjusted(self):
        editor = self.drag_editor()
        editor.drawing = False
        editor.last_point = None

        editor.mouse(cv2.EVENT_LBUTTONDOWN, 5, 10, 0, None)
        self.assertFalse(editor.drawing)
        editor.mouse(cv2.EVENT_LBUTTONDOWN, 5, HEADER_HEIGHT + 3, 0, None)
        self.assertEqual(editor.label[3, 5], 4)

    def test_mouse_move_without_button_clears_stale_drag(self):
        editor = self.drag_editor(button_flag_seen=True)

        editor.mouse(cv2.EVENT_MOUSEMOVE, 5, HEADER_HEIGHT + 5, 0, None)
        self.assertFalse(editor.drawing)
        self.assertIsNone(editor.last_point)
        self.assertEqual(editor.label[5, 5], 0)

    def test_mouse_move_with_button_continues_drag(self):
        editor = self.drag_editor()

        editor.mouse(cv2.EVENT_MOUSEMOVE, 5, HEADER_HEIGHT + 5,
                     cv2.EVENT_FLAG_LBUTTON, None)
        self.assertTrue(editor.drawing)
        self.assertTrue(editor.button_flag_seen)
        self.assertEqual(editor.label[5, 5], 4)

    def test_drag_paints_when_backend_never_reports_button_flag(self):
        # GTK3 builds do not always set EVENT_FLAG_LBUTTON on move events.
        # Cancelling the drag there would make the brush useless.
        editor = self.drag_editor()

        editor.mouse(cv2.EVENT_MOUSEMOVE, 5, HEADER_HEIGHT + 5, 0, None)
        self.assertTrue(editor.drawing)
        self.assertEqual(editor.label[5, 5], 4)

    def drag_editor(self, button_flag_seen=False):
        editor = LabelEditor.__new__(LabelEditor)
        editor.label = np.zeros((10, 10), dtype=np.uint8)
        editor.class_id = 4
        editor.mode = 'brush'
        editor.brush_size = 1
        editor.drawing = True
        editor.button_flag_seen = button_flag_seen
        editor.last_point = (1, 1)
        editor.undo_stack = []
        editor.roi = np.full((10, 10), 255, dtype=np.uint8)
        return editor

    def test_paint_outside_roi_is_discarded(self):
        editor = self.drag_editor()
        editor.label = np.zeros((360, 640), dtype=np.uint8)
        editor.roi = roi_mask(editor.label.shape)

        # 라벨링 범위 위쪽(y=100)은 남지 않고, 안쪽(y=300)은 남아야 한다.
        editor.mouse(cv2.EVENT_LBUTTONDOWN, 320, HEADER_HEIGHT + 100, 0, None)
        self.assertEqual(editor.label[100, 320], 0)
        editor.mouse(cv2.EVENT_LBUTTONDOWN, 320, HEADER_HEIGHT + 300, 0, None)
        self.assertEqual(editor.label[300, 320], 4)

    def test_stroke_across_roi_edge_keeps_only_inside(self):
        editor = self.drag_editor()
        editor.label = np.zeros((360, 640), dtype=np.uint8)
        editor.roi = roi_mask(editor.label.shape)
        editor.brush_size = 5
        editor.last_point = (320, 200)

        editor.mouse(cv2.EVENT_MOUSEMOVE, 320, HEADER_HEIGHT + 230,
                     cv2.EVENT_FLAG_LBUTTON, None)
        painted_rows = np.where((editor.label == 4).any(axis=1))[0]
        self.assertTrue(painted_rows.size)
        self.assertGreaterEqual(int(painted_rows.min()), 216)

    def test_print_controls_lists_required_keys_and_paths(self):
        output = StringIO()
        with redirect_stdout(output):
            print_controls('/tmp/dataset', 'train', 250)
        text = output.getvalue()
        for expected in ('250장', '브러시 모드', '연결 컴포넌트',
                         '되돌리기', '저장 후 종료', 'labels/train', 'previews/train'):
            self.assertIn(expected, text)

    def test_save_updates_label_and_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image_path = root / 'images/train/frame.png'
            label_path = root / 'labels/train/frame.png'
            image_path.parent.mkdir(parents=True)
            label_path.parent.mkdir(parents=True)
            image = np.full((12, 16, 3), 100, dtype=np.uint8)
            label = np.full((12, 16), 4, dtype=np.uint8)
            cv2.imwrite(str(image_path), image)
            old_preview = root / 'previews/train/frame.png'
            old_preview.parent.mkdir(parents=True)
            cv2.imwrite(str(old_preview), image)

            editor = LabelEditor.__new__(LabelEditor)
            editor.root = root
            editor.split = 'train'
            editor.items = [(image_path, label_path)]
            editor.index = 0
            editor.image = image
            editor.label = label
            editor.config = None
            editor.save()

            saved = cv2.imread(str(label_path), cv2.IMREAD_GRAYSCALE)
            preview = root / 'previews/train/frame.jpg'
            self.assertTrue(np.array_equal(saved, label))
            self.assertTrue(preview.exists())
            self.assertFalse(old_preview.exists())
            self.assertEqual(len(list(preview.parent.glob('frame.*'))), 1)

    def fill_editor(self):
        editor = self.drag_editor()
        editor.label = np.zeros((360, 640), dtype=np.uint8)
        editor.roi = roi_mask(editor.label.shape)
        editor.mode = 'fill'
        editor.tolerance = 8
        editor.image = np.zeros((360, 640, 3), dtype=np.uint8)
        # 왼쪽은 짙은 회색, 오른쪽은 밝은 회색. 허용 오차보다 훨씬 차이 난다.
        editor.image[:, :320] = 40
        editor.image[:, 320:] = 200
        return editor

    def test_fill_spreads_over_similar_colour_only(self):
        editor = self.fill_editor()
        editor.apply_fill(100, 300)
        self.assertTrue(np.all(editor.label[216:, :320] == 4))
        self.assertTrue(np.all(editor.label[216:, 320:] == 0))

    def test_fill_stays_inside_label_roi(self):
        editor = self.fill_editor()
        editor.apply_fill(100, 300)
        self.assertTrue(np.all(editor.label[:216] == 0))

    def test_fill_does_not_modify_the_image(self):
        editor = self.fill_editor()
        before = editor.image.copy()
        editor.apply_fill(100, 300)
        self.assertTrue(np.array_equal(before, editor.image))

    def test_fill_outside_roi_paints_nothing(self):
        editor = self.fill_editor()
        editor.apply_fill(100, 100)
        self.assertEqual(int((editor.label > 0).sum()), 0)

    def test_label_roi_is_bottom_40_percent(self):
        self.assertEqual(label_roi_top(360), 216)
        mask = roi_mask((360, 640))
        self.assertEqual(int((mask > 0).sum()) / mask.size, 0.4)

    def test_label_roi_covers_lane_detection_roi(self):
        # 라벨링 범위가 런타임 ROI 를 덮어야 한다. 런타임 ROI 를 여기보다 위로 올리면
        # 라벨 없는 영역을 쓰게 되므로 이 테스트가 먼저 깨진다.
        polygon = lane_detection_roi_polygon((360, 640))
        self.assertEqual([list(point) for point in polygon],
                         [[32, 251], [608, 251], [960, 360], [-320, 360]])
        self.assertGreaterEqual(int(polygon[:, 1].min()), label_roi_top(360))

    def test_clear_outside_roi_keeps_inside_and_reports_count(self):
        label = np.full((360, 640), 1, dtype=np.uint8)
        changed = clear_outside_roi(label)
        mask = roi_mask(label.shape)
        self.assertEqual(changed, int((mask == 0).sum()))
        self.assertTrue(np.all(label[mask > 0] == 1))
        self.assertTrue(np.all(label[:216] == 0))

    def test_undo_restores_previous_label(self):
        editor = LabelEditor.__new__(LabelEditor)
        editor.label = np.ones((2, 2), dtype=np.uint8)
        editor.undo_stack = [np.zeros((2, 2), dtype=np.uint8)]
        editor.undo()
        self.assertTrue(np.all(editor.label == 0))
        self.assertFalse(editor.undo_stack)
