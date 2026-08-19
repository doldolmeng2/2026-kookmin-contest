import unittest
import torch
from segmentation_tools.criterion import CrossEntropy, OhemCrossEntropy, BondaryLoss


class CriterionTest(unittest.TestCase):
    def test_ohem_balance_weighted_list_needs_external_mean(self):
        # The aux-head branch keeps reduction='none' (matches upstream
        # utils/criterion.py), so the balance-weighted sum broadcasts to a
        # per-pixel map, not a scalar; callers reduce it themselves, exactly
        # like utils/function.py:train()'s `loss = losses.mean()`.
        aux=torch.randn(2,3,4,4); main=torch.randn(2,3,4,4); target=torch.randint(0,3,(2,4,4))
        criterion=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1,balance_weights=(0.4,1.0))
        loss=criterion([aux,main],target)
        self.assertEqual(tuple(loss.shape),(2,4,4))
        self.assertTrue(torch.isfinite(loss.mean()))

    def test_ohem_single_tensor_uses_sb_weight(self):
        main=torch.randn(2,3,4,4); target=torch.randint(0,3,(2,4,4))
        doubled=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1,sb_weights=2.0)(main,target)
        single=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1,sb_weights=1.0)(main,target)
        self.assertAlmostEqual(float(doubled),2*float(single),places=4)

    def test_ohem_masks_out_ignored_pixels(self):
        main=torch.randn(1,2,3,3); target=torch.zeros(1,3,3,dtype=torch.int64); target[:,0,:]=255
        criterion=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1)
        loss=criterion(main,target)
        self.assertTrue(torch.isfinite(loss))
        all_valid_loss=criterion(main,torch.zeros(1,3,3,dtype=torch.int64))
        self.assertNotAlmostEqual(float(loss),float(all_valid_loss),places=4)

    def test_class_weight_changes_loss(self):
        torch.manual_seed(0)
        main=torch.randn(1,2,4,4); target=torch.zeros(1,4,4,dtype=torch.int64)
        unweighted=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1)(main,target)
        weighted=OhemCrossEntropy(ignore_label=255,thres=.7,min_kept=1,weight=torch.tensor([5.,1.]))(main,target)
        self.assertNotAlmostEqual(float(unweighted),float(weighted),places=3)

    def test_cross_entropy_balance_weighted(self):
        aux=torch.randn(2,3,4,4); main=torch.randn(2,3,4,4); target=torch.randint(0,3,(2,4,4))
        criterion=CrossEntropy(ignore_label=255,balance_weights=(0.4,1.0))
        loss=criterion([aux,main],target)
        self.assertEqual(loss.dim(),0); self.assertTrue(torch.isfinite(loss))

    def test_boundary_loss_prefers_matching_prediction(self):
        target=torch.zeros(1,1,4,4); target[:,:,1:3,1:3]=1.
        matching=torch.full((1,1,4,4),-10.); matching[:,:,1:3,1:3]=10.
        opposite=torch.full((1,1,4,4),10.); opposite[:,:,1:3,1:3]=-10.
        loss=BondaryLoss()
        self.assertGreater(float(loss(opposite,target)),float(loss(matching,target)))


if __name__=='__main__': unittest.main()
