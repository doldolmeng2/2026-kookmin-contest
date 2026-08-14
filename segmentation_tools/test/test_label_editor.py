import tempfile
import unittest
from pathlib import Path
from contextlib import redirect_stdout
from io import StringIO

import cv2
import numpy as np

from segmentation_tools.label_editor import LabelEditor, print_controls


class LabelEditorTest(unittest.TestCase):
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

    def test_undo_restores_previous_label(self):
        editor = LabelEditor.__new__(LabelEditor)
        editor.label = np.ones((2, 2), dtype=np.uint8)
        editor.undo_stack = [np.zeros((2, 2), dtype=np.uint8)]
        editor.undo()
        self.assertTrue(np.all(editor.label == 0))
        self.assertFalse(editor.undo_stack)
