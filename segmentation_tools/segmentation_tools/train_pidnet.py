"""Dataset adapter and trainer for the official XuJiacong/PIDNet PIDNet-S.

Loss composition mirrors the official utils/utils.py:FullModel.forward and
utils/criterion.py at commit 4c158cf24ce432f0a8cb43364fae38d93cee0dc3:
loss = loss_s (balance-weighted aux/main OHEM) + loss_b (boundary BCE)
     + loss_sb (main-head OHEM restricted to confident-boundary pixels).
"""
import argparse, json, random, time
from pathlib import Path
import cv2, numpy as np, torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from .pidnet import PIDNet
from .lighting_augmentation import augment_lighting
from .criterion import CrossEntropy, OhemCrossEntropy, BondaryLoss


def make_pidnet_s(classes, augment=True):
    """Official PIDNet-S parameters from models/pidnet.py:get_seg_model."""
    return PIDNet(m=2,n=3,num_classes=classes,planes=32,ppm_planes=96,head_planes=128,augment=augment)


def load_imagenet_pretrained(model, path):
    """Port of pidnet.py:get_seg_model's imgnet_pretrained=True branch: load
    every backbone tensor whose name and shape match; segmentation heads
    (shaped by num_classes) are silently skipped."""
    state = torch.load(path, map_location='cpu')
    state = state['state_dict'] if 'state_dict' in state else state
    model_dict = model.state_dict()
    matched = {k: v for k, v in state.items() if k in model_dict and v.shape == model_dict[k].shape}
    model_dict.update(matched)
    model.load_state_dict(model_dict, strict=False)
    return len(matched), len(model_dict)


def compute_class_weights(rows, classes, ignore, clip=(0.1, 10.0)):
    """Median-frequency balancing over the train split's label pixels, in the
    spirit of the fixed class_weights tensor datasets/cityscapes.py passes into
    OhemCrossEntropy/CrossEntropy. Cityscapes' weights are dataset-specific
    constants; this project has a different, far more skewed class mix, so the
    weights are computed from this dataset's own pixel histogram instead of
    reused verbatim. Clipped to keep OHEM+AMP stable under this dataset's
    extreme imbalance (rare lane classes at ~1-2% of pixels)."""
    counts = np.zeros(classes, dtype=np.int64)
    for _, label_path in rows:
        label = cv2.imread(str(label_path), 0)
        if label is None: raise OSError(f'cannot read {label_path}')
        valid = label != ignore
        counts += np.bincount(label[valid].ravel(), minlength=classes)[:classes]
    total = counts.sum()
    if total == 0: raise ValueError('no valid (non-ignore) label pixels found to compute class weights')
    freq = counts / total
    present = freq > 0
    median_freq = np.median(freq[present])
    weights = np.ones(classes, dtype=np.float64)
    weights[present] = median_freq / freq[present]
    weights = np.clip(weights, clip[0], clip[1])
    return torch.tensor(weights, dtype=torch.float32), counts.tolist()


def read_list(root,split):
    path=root/'lists'/f'{split}.lst'
    if not path.exists(): return []
    rows=[]
    for number,line in enumerate(path.read_text(encoding='utf-8').splitlines(),1):
        if not line.strip(): continue
        fields=line.split()
        if len(fields)<2: raise ValueError(f'{path}:{number}: expected IMAGE LABEL')
        rows.append((root/fields[0],root/fields[1]))
    return rows


class SegDataset(Dataset):
    def __init__(self,rows,size=None,augment=False): self.rows,self.size,self.augment=list(rows),size,augment
    def __len__(self): return len(self.rows)
    def __getitem__(self,index):
        image_path,label_path=self.rows[index]; image=cv2.imread(str(image_path)); label=cv2.imread(str(label_path),0)
        if image is None or label is None: raise OSError(f'cannot read {image_path} or {label_path}')
        if image.shape[:2]!=label.shape: raise ValueError(f'size mismatch: {image_path}, {label_path}')
        if self.size and image.shape[1::-1]!=self.size:
            image=cv2.resize(image,self.size); label=cv2.resize(label,self.size,interpolation=cv2.INTER_NEAREST)
        if self.augment: image=augment_lighting(image)
        image=cv2.cvtColor(image.astype(np.uint8),cv2.COLOR_BGR2RGB).astype(np.float32)/255
        image=(image-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        return torch.from_numpy(image.transpose(2,0,1).copy()),torch.from_numpy(label.astype(np.int64))


def resize(score,target): return nn.functional.interpolate(score,target.shape[-2:],mode='bilinear',align_corners=False)


def boundary_target(label):
    """Official datasets/cityscapes.py builds this per-sample via cv2.Canny on
    the label image; this is a batch-vectorized equivalent (any 4-neighbour
    class transition is a boundary pixel), then dilated like upstream."""
    edge=torch.zeros_like(label,dtype=torch.bool)
    edge[:,1:]|=label[:,1:]!=label[:,:-1]; edge[:,:-1]|=label[:,:-1]!=label[:,1:]
    edge[:,:,1:]|=label[:,:,1:]!=label[:,:,:-1]; edge[:,:,:-1]|=label[:,:,:-1]!=label[:,:,1:]
    return nn.functional.max_pool2d(edge.float().unsqueeze(1),3,1,1)


def run_epoch(model,loader,device,optimizer,scaler,classes,ignore,training,sem_criterion,bd_criterion):
    model.train(training); matrix=torch.zeros(classes,classes,dtype=torch.int64,device=device); total=0.
    for images,labels in loader:
        images,labels=images.to(device,non_blocking=True),labels.to(device,non_blocking=True)
        if training: optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training),torch.autocast(device_type=device.type,enabled=scaler.is_enabled()):
            auxiliary,main,boundary=model(images); auxiliary,main,boundary=resize(auxiliary,labels),resize(main,labels),resize(boundary,labels)
            edge=boundary_target(labels)
            loss_s=sem_criterion([auxiliary,main],labels)
            loss_b=bd_criterion(boundary,edge)
            filler=torch.full_like(labels,ignore)
            bd_label=torch.where(boundary[:,0].sigmoid()>.8,labels,filler)
            loss_sb=sem_criterion(main,bd_label)
            # OhemCrossEntropy's non-OHEM branch (used for the aux head inside loss_s)
            # keeps its internal criterion at reduction='none', so loss_s is a per-pixel
            # map, not a scalar; official utils/function.py:train() reduces the same way
            # via `loss = losses.mean()` after FullModel.forward.
            loss=(loss_s+loss_b+loss_sb).mean()
        if training: scaler.scale(loss).backward(); scaler.step(optimizer); scaler.update()
        total+=float(loss.detach())*len(images); prediction=main.argmax(1); keep=labels!=ignore
        matrix+=torch.bincount(labels[keep]*classes+prediction[keep],minlength=classes**2).reshape(classes,classes)
    intersection=matrix.diag().double(); union=matrix.sum(0)+matrix.sum(1)-intersection; iou=torch.where(union>0,intersection/union,torch.nan)
    return {'loss':total/len(loader.dataset),'miou':float(torch.nanmean(iou)),'accuracy':float(intersection.sum()/matrix.sum().clamp_min(1)),'class_iou':[None if torch.isnan(x) else float(x) for x in iou]}


def size_arg(text):
    try: width,height=map(int,text.lower().split('x'))
    except ValueError as error: raise argparse.ArgumentTypeError('use WIDTHxHEIGHT') from error
    if width%8 or height%8: raise argparse.ArgumentTypeError('width and height must be divisible by 8')
    return width,height


def main(argv=None):
    parser=argparse.ArgumentParser(description='Train official PIDNet-S on a list dataset'); parser.add_argument('dataset',type=Path); parser.add_argument('--output',type=Path,default=Path('runs/pidnet_s')); parser.add_argument('--epochs',type=int,default=200); parser.add_argument('--batch-size',type=int,default=6); parser.add_argument('--workers',type=int,default=3); parser.add_argument('--lr',type=float,default=.005); parser.add_argument('--weight-decay',type=float,default=.0005); parser.add_argument('--momentum',type=float,default=.9); parser.add_argument('--image-size',type=size_arg); parser.add_argument('--val-fraction',type=float,default=.1); parser.add_argument('--ohem-threshold',type=float,default=.9); parser.add_argument('--ohem-keep',type=int,default=131072); parser.add_argument('--no-ohem',action='store_true',help='use plain CrossEntropy instead of OHEM for the main head (official LOSS.USE_OHEM=false)'); parser.add_argument('--pretrained',type=Path,help='official ImageNet-pretrained PIDNet-S backbone (e.g. PIDNet_S_ImageNet.pth.tar)'); parser.add_argument('--no-class-weights',action='store_true',help='disable median-frequency class weighting (official uses a fixed per-dataset class_weights tensor)'); parser.add_argument('--device',default='auto'); parser.add_argument('--resume',type=Path); parser.add_argument('--seed',type=int,default=42); parser.add_argument('--no-amp',action='store_true'); parser.add_argument('--patience',type=int,default=25,help='early-stop after this many epochs without val mIoU improvement; 0 disables'); args=parser.parse_args(argv)
    if args.batch_size<2: raise ValueError('official PIDNet-S training needs --batch-size >= 2 for BatchNorm')
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed); root=args.dataset.expanduser().resolve(); metadata={}
    if (root/'dataset.yaml').exists():
        import yaml; metadata=yaml.safe_load((root/'dataset.yaml').read_text(encoding='utf-8')) or {}
    classes,ignore=int(metadata.get('num_classes',6)),int(metadata.get('ignore_label',255)); train,val,test=read_list(root,'train'),read_list(root,'val'),read_list(root,'test')
    if not train: raise ValueError(f'no samples in {root}/lists/train.lst')
    if not test: raise ValueError(f'no samples in {root}/lists/test.lst; test set is required and must remain held out')
    if not val:
        if not 0<args.val_fraction<1: raise ValueError('empty val.lst requires 0 < --val-fraction < 1')
        random.Random(args.seed).shuffle(train); count=max(1,round(len(train)*args.val_fraction)); val,train=train[:count],train[count:]
    device=torch.device(('cuda' if torch.cuda.is_available() else 'cpu') if args.device=='auto' else args.device); pin=device.type=='cuda'
    train_loader=DataLoader(SegDataset(train,args.image_size,True),args.batch_size,shuffle=True,num_workers=args.workers,pin_memory=pin,drop_last=(len(train)%args.batch_size==1)); val_loader=DataLoader(SegDataset(val,args.image_size),args.batch_size,num_workers=args.workers,pin_memory=pin); test_loader=DataLoader(SegDataset(test,args.image_size),args.batch_size,num_workers=args.workers,pin_memory=pin)
    model=make_pidnet_s(classes,True)
    pretrained_report=None
    if args.pretrained:
        matched,total=load_imagenet_pretrained(model,args.pretrained.expanduser())
        pretrained_report=f'{matched}/{total} tensors'; print(f'pretrained backbone: loaded {pretrained_report} from {args.pretrained}')
    model=model.to(device)
    class_weights,class_counts=(None,None)
    if not args.no_class_weights:
        class_weights,class_counts=compute_class_weights(train,classes,ignore)
        print(f'class weights (median-frequency balanced): {[round(w,3) for w in class_weights.tolist()]}')
        class_weights=class_weights.to(device)
    criterion_cls=CrossEntropy if args.no_ohem else OhemCrossEntropy
    criterion_kwargs=dict(ignore_label=ignore,weight=class_weights,balance_weights=(0.4,1.0),sb_weights=1.0)
    if not args.no_ohem: criterion_kwargs.update(thres=args.ohem_threshold,min_kept=args.ohem_keep)
    sem_criterion=criterion_cls(**criterion_kwargs); bd_criterion=BondaryLoss()
    optimizer=torch.optim.SGD(model.parameters(),lr=args.lr,momentum=args.momentum,weight_decay=args.weight_decay); scheduler=torch.optim.lr_scheduler.PolynomialLR(optimizer,args.epochs,power=.9); scaler=torch.amp.GradScaler('cuda',enabled=pin and not args.no_amp); start,best,stale=1,-1.,0
    if args.resume:
        state=torch.load(args.resume.expanduser(),map_location=device,weights_only=False); model.load_state_dict(state['model']); optimizer.load_state_dict(state['optimizer']); scheduler.load_state_dict(state['scheduler']); start,best=state['epoch']+1,state.get('best_miou',-1.)
    args.output.mkdir(parents=True,exist_ok=True); config=vars(args).copy(); config.update(dataset=str(root),output=str(args.output),resume=str(args.resume) if args.resume else None,pretrained=str(args.pretrained) if args.pretrained else None,pretrained_report=pretrained_report,image_size=args.image_size,classes=classes,ignore=ignore,train=len(train),val=len(val),test=len(test),device=str(device),ohem=not args.no_ohem,class_weights=None if class_weights is None else class_weights.tolist(),class_pixel_counts=class_counts,upstream='https://github.com/XuJiacong/PIDNet',upstream_commit='4c158cf24ce432f0a8cb43364fae38d93cee0dc3'); (args.output/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8'); print(f'device={device} train={len(train)} val={len(val)} test={len(test)} classes={classes}')
    for epoch in range(start,args.epochs+1):
        then=time.time(); train_result=run_epoch(model,train_loader,device,optimizer,scaler,classes,ignore,True,sem_criterion,bd_criterion); val_result=run_epoch(model,val_loader,device,optimizer,scaler,classes,ignore,False,sem_criterion,bd_criterion); scheduler.step(); state={'epoch':epoch,'model':model.state_dict(),'optimizer':optimizer.state_dict(),'scheduler':scheduler.state_dict(),'best_miou':max(best,val_result['miou']),'config':config}; torch.save(state,args.output/'last.pt')
        if val_result['miou']>best:
            best=val_result['miou']; stale=0; state['best_miou']=best; torch.save(state,args.output/'best.pt')
        else: stale+=1
        record={'epoch':epoch,'seconds':time.time()-then,'train':train_result,'val':val_result}; stream=(args.output/'metrics.jsonl').open('a',encoding='utf-8'); stream.write(json.dumps(record)+'\n'); stream.close(); print(f"[{epoch:03d}/{args.epochs}] train loss={train_result['loss']:.4f} mIoU={train_result['miou']:.4f} | val loss={val_result['loss']:.4f} mIoU={val_result['miou']:.4f} acc={val_result['accuracy']:.4f} | {record['seconds']:.1f}s")
        if args.patience>0 and stale>=args.patience:
            print(f'early stopping: val mIoU did not improve for {stale} epochs; best={best:.4f}')
            break

    best_state=torch.load(args.output/'best.pt',map_location=device,weights_only=False)
    model.load_state_dict(best_state['model'])
    test_result=run_epoch(model,test_loader,device,optimizer,scaler,classes,ignore,False,sem_criterion,bd_criterion)
    test_record={'checkpoint':'best.pt','best_epoch':best_state['epoch'],'best_val_miou':best_state['best_miou'],'test':test_result}
    (args.output/'test_metrics.json').write_text(json.dumps(test_record,indent=2),encoding='utf-8')
    print(f"test best_epoch={best_state['epoch']} loss={test_result['loss']:.4f} mIoU={test_result['miou']:.4f} acc={test_result['accuracy']:.4f}")
if __name__=='__main__': main()