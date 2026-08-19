"""Loss functions ported from the official XuJiacong/PIDNet utils/criterion.py.

Repository: https://github.com/XuJiacong/PIDNet
Commit: 4c158cf24ce432f0a8cb43364fae38d93cee0dc3
Adapted to take balance_weights/sb_weights as constructor arguments instead of
reading them from the upstream yacs `config` singleton, since this project has
no global config object. The math is unchanged.
"""
import torch
from torch import nn
from torch.nn import functional as F


class CrossEntropy(nn.Module):
    def __init__(self, ignore_label=-1, weight=None, balance_weights=(0.4, 1.0), sb_weights=1.0):
        super().__init__()
        self.ignore_label = ignore_label
        self.balance_weights = balance_weights
        self.sb_weights = sb_weights
        self.criterion = nn.CrossEntropyLoss(weight=weight, ignore_index=ignore_label)

    def _forward(self, score, target):
        return self.criterion(score, target)

    def forward(self, score, target):
        if not isinstance(score, (list, tuple)):
            score = [score]
        if len(self.balance_weights) == len(score):
            return sum(w * self._forward(x, target) for w, x in zip(self.balance_weights, score))
        if len(score) == 1:
            return self.sb_weights * self._forward(score[0], target)
        raise ValueError('lengths of prediction and target are not identical!')


class OhemCrossEntropy(nn.Module):
    def __init__(self, ignore_label=-1, thres=0.7, min_kept=100000, weight=None,
                 balance_weights=(0.4, 1.0), sb_weights=1.0):
        super().__init__()
        self.thresh = thres
        self.min_kept = max(1, min_kept)
        self.ignore_label = ignore_label
        self.balance_weights = balance_weights
        self.sb_weights = sb_weights
        self.criterion = nn.CrossEntropyLoss(weight=weight, ignore_index=ignore_label, reduction='none')

    def _ce_forward(self, score, target):
        return self.criterion(score, target)

    def _ohem_forward(self, score, target):
        pred = F.softmax(score, dim=1)
        pixel_losses = self.criterion(score, target).contiguous().view(-1)
        mask = target.contiguous().view(-1) != self.ignore_label

        tmp_target = target.clone()
        tmp_target[tmp_target == self.ignore_label] = 0
        pred = pred.gather(1, tmp_target.unsqueeze(1))
        pred, ind = pred.contiguous().view(-1)[mask].contiguous().sort()
        min_value = pred[min(self.min_kept, pred.numel() - 1)]
        threshold = max(min_value, self.thresh)

        pixel_losses = pixel_losses[mask][ind]
        pixel_losses = pixel_losses[pred < threshold]
        return pixel_losses.mean() if pixel_losses.numel() else score.sum() * 0

    def forward(self, score, target):
        if not isinstance(score, (list, tuple)):
            score = [score]
        if len(self.balance_weights) == len(score):
            functions = [self._ce_forward] * (len(self.balance_weights) - 1) + [self._ohem_forward]
            return sum(w * func(x, target) for w, x, func in zip(self.balance_weights, score, functions))
        if len(score) == 1:
            return self.sb_weights * self._ohem_forward(score[0], target)
        raise ValueError('lengths of prediction and target are not identical!')


def weighted_bce(bd_pre, target):
    n, c, h, w = bd_pre.size()
    log_p = bd_pre.permute(0, 2, 3, 1).contiguous().view(1, -1)
    target_t = target.view(1, -1)

    pos_index = target_t == 1
    neg_index = target_t == 0

    weight = torch.zeros_like(log_p)
    pos_num = pos_index.sum()
    neg_num = neg_index.sum()
    sum_num = pos_num + neg_num
    weight[pos_index] = neg_num * 1.0 / sum_num
    weight[neg_index] = pos_num * 1.0 / sum_num

    return F.binary_cross_entropy_with_logits(log_p, target_t, weight, reduction='mean')


class BondaryLoss(nn.Module):
    def __init__(self, coeff_bce=20.0):
        super().__init__()
        self.coeff_bce = coeff_bce

    def forward(self, bd_pre, bd_gt):
        return self.coeff_bce * weighted_bce(bd_pre, bd_gt)
