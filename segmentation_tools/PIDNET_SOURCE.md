# PIDNet upstream source

The model implementation in `segmentation_tools/pidnet.py` and
`segmentation_tools/model_utils.py` is copied from the official PIDNet repository:

- Repository: https://github.com/XuJiacong/PIDNet
- Commit: `4c158cf24ce432f0a8cb43364fae38d93cee0dc3`
- Author: Jiacong Xu
- License: MIT; see `third_party/PIDNet_LICENSE`

`segmentation_tools/criterion.py` is likewise ported from the official
`utils/criterion.py` at the same commit (`CrossEntropy`, `OhemCrossEntropy`,
`weighted_bce`, `BondaryLoss`), adapted to take `balance_weights`/`sb_weights`
as constructor arguments instead of a global yacs `config` object.

`train_pidnet.py` is a project-specific dataset/training adapter, but its loss
composition now mirrors the official `utils/utils.py:FullModel.forward` and
`tools/train.py` exactly:
- `loss_s`: `OhemCrossEntropy([aux, main], labels)` with balance weights `[0.4, 1.0]`
  (plain CE on the aux head, OHEM on the main head — `LOSS.OHEMTHRES=0.9`,
  `LOSS.OHEMKEEP=131072`).
- `loss_b`: `BondaryLoss(main_boundary_head, edge_target)`, coefficient 20.
- `loss_sb`: `OhemCrossEntropy(main, bd_label)` where `bd_label` masks the label to
  `ignore` everywhere the boundary head isn't confident (`sigmoid > 0.8`) — the
  official "boundary-aware" semantic loss term.
- `loss = (loss_s + loss_b + loss_sb).mean()`, matching `utils/function.py:train()`'s
  `loss = losses.mean()` (needed because `OhemCrossEntropy`'s aux-head branch uses
  `reduction='none'` internally, same as upstream).

Two things the official recipe supports and this project now wires up, since they
were the most likely cause of PIDNet-S under-fitting the lane-marking classes:
- **Class weights**: `train_pidnet.py:compute_class_weights` computes a
  median-frequency-balanced weight vector from the train split's own label pixel
  histogram (clipped to `[0.1, 10.0]` for OHEM/AMP stability) and passes it into
  `OhemCrossEntropy(weight=...)`, in the same place `datasets/cityscapes.py`'s fixed
  `class_weights` tensor is used upstream. Disable with `--no-class-weights`.
- **ImageNet pretraining**: `--pretrained PATH` loads an official ImageNet-pretrained
  PIDNet-S backbone checkpoint (e.g. `PIDNet_S_ImageNet.pth.tar` from the PIDNet
  releases) via the same key/shape-matching logic as `pidnet.py:get_seg_model`'s
  `imgnet_pretrained=True` branch; the two segmentation heads (shaped by
  `num_classes`) are skipped and stay randomly initialized. Not required.

Not ported (out of scope — this project isn't Cityscapes and runs on one GPU):
Cityscapes' 19-class label mapping, multi-scale/random-crop/flip augmentation
(this camera's fixed viewpoint and left/right-solid-lane classes make naive flip
augmentation unsafe without also swapping those two labels), the yacs config
system, and `DataParallel`/distributed training.