import tempfile
import unittest
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from segmentation_tools.split_dataset import assign, parse_part, split


class AssignTest(unittest.TestCase):
    def test_each_part_gets_requested_count(self):
        parts = [('서형찬', 50), ('남궁길', 67), ('구민관', 67), ('이찬영', 66)]
        assignment = assign(250, parts)
        self.assertEqual(Counter(assignment), Counter(dict(parts)))

    def test_assignment_is_interleaved_not_contiguous(self):
        parts = [('a', 50), ('b', 67), ('c', 67), ('d', 66)]
        assignment = assign(250, parts)
        # 각 담당자의 몫이 앞뒤 어느 한쪽에 쏠리지 않아야 한다. 앞 절반에 들어간
        # 비율이 절반 언저리인지로 확인한다.
        for name, size in parts:
            first_half = sum(1 for value in assignment[:125] if value == name)
            self.assertAlmostEqual(first_half / size, 0.5, delta=0.05)

    def test_mismatched_total_is_rejected(self):
        with self.assertRaises(ValueError):
            assign(250, [('a', 100), ('b', 100)])

    def test_parse_part_reads_name_and_count(self):
        self.assertEqual(parse_part('서형찬=50'), ('서형찬', 50))


class SplitTest(unittest.TestCase):
    def test_split_copies_frames_and_writes_lists(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'batch'
            for index in range(4):
                for folder, image in (('images', np.zeros((360, 640, 3), np.uint8)),
                                      ('labels', np.zeros((360, 640), np.uint8))):
                    path = root / folder / 'train' / f'frame_{index}.png'
                    path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(path), image)
            lists = root / 'lists'
            lists.mkdir(parents=True)
            (lists / 'train.lst').write_text(''.join(
                f'images/train/frame_{index}.png labels/train/frame_{index}.png\n'
                for index in range(4)), encoding='utf-8')

            outputs = split(root, Path(directory) / 'out', [('갑', 3), ('을', 1)])

            self.assertEqual([count for _, count in outputs], [3, 1])
            for output, count in outputs:
                names = (output / 'lists' / 'train.lst').read_text(encoding='utf-8').split()
                self.assertEqual(len(names), count * 2)
                self.assertEqual(len(list((output / 'images' / 'train').glob('*.png'))), count)
                self.assertEqual(len(list((output / 'labels' / 'train').glob('*.png'))), count)
                self.assertTrue((output / 'classes.json').exists())
                self.assertTrue((output / 'dataset.yaml').exists())
