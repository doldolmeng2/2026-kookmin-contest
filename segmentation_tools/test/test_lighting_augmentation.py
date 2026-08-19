import unittest
import numpy as np
from segmentation_tools.lighting_augmentation import augment_lighting


class LightingAugmentationTest(unittest.TestCase):
    def test_preserves_shape_dtype_and_input(self):
        image=np.full((80,120,3),128,np.uint8); original=image.copy()
        result=augment_lighting(image,np.random.default_rng(7))
        self.assertEqual(result.shape,image.shape); self.assertEqual(result.dtype,np.uint8)
        self.assertTrue(np.array_equal(image,original))

    def test_is_deterministic_with_rng_and_brightens_and_darkens(self):
        image=np.full((80,120,3),128,np.uint8)
        self.assertTrue(np.array_equal(augment_lighting(image,np.random.default_rng(11)),augment_lighting(image,np.random.default_rng(11))))
        means=[augment_lighting(image,np.random.default_rng(seed)).mean() for seed in range(40)]
        self.assertLess(min(means),image.mean()); self.assertGreater(max(means),image.mean())


if __name__=='__main__': unittest.main()