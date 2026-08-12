import argparse
import json
from pathlib import Path

import cv2
import yaml

from .bag_source import BagFrameSource
from .core import CLASS_NAMES, default_config_path, generate_label, load_config, overlay


def write_sample(root, split, stem, image, label, config, preview):
    image_rel = Path('images') / split / f'{stem}.png'
    label_rel = Path('labels') / split / f'{stem}.png'
    image_path, label_path = root / image_rel, root / label_rel
    image_path.parent.mkdir(parents=True, exist_ok=True)
    label_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(image_path), image):
        raise OSError(f'이미지 저장 실패: {image_path}')
    if not cv2.imwrite(str(label_path), label):
        raise OSError(f'라벨 저장 실패: {label_path}')
    if preview:
        preview_path = root / 'previews' / split / f'{stem}.jpg'
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(preview_path), overlay(image, label, config, 0.5))
    return image_rel.as_posix(), label_rel.as_posix()


def extract(args):
    source = BagFrameSource(args.bag, args.topic)
    config = load_config(args.config)
    root = Path(args.output).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    if not 0 <= args.start_index < len(source):
        raise ValueError(f'--start-index는 0~{len(source) - 1} 범위여야 합니다.')
    end_index = len(source) - 1 if args.end_index < 0 else args.end_index
    if not args.start_index <= end_index < len(source):
        raise ValueError(
            f'--end-index는 {args.start_index}~{len(source) - 1} 범위여야 합니다.')
    start_seconds = source.seconds(args.start_index)
    end_seconds = source.seconds(end_index)
    if args.uniform and args.count > 0:
        if args.count == 1:
            indices = [args.start_index]
        else:
            span = end_index - args.start_index
            indices = [args.start_index + round(span * value / (args.count - 1))
                       for value in range(args.count)]
            indices = list(dict.fromkeys(indices))
    else:
        available = max(0.0, end_seconds - start_seconds)
        target_count = int(available / args.interval) + 1
        if args.count > 0:
            target_count = min(target_count, args.count)
        targets = [start_seconds + value * args.interval for value in range(target_count)]
        indices = list(dict.fromkeys(source.nearest_index(seconds) for seconds in targets))
    lists = {'train': [], 'val': []}
    bag_name = Path(args.bag).resolve().name.replace('rosbag2_', '')
    for number, index in enumerate(indices):
        image = source.image(index)
        label = generate_label(image, config)
        background_rows = int(round(label.shape[0] * args.top_background))
        if background_rows > 0:
            label[:background_rows, :] = 0
        # Stable periodic split keeps extraction reproducible without shuffling time order.
        split = 'val' if args.val_every > 0 and number % args.val_every == 0 else 'train'
        stem = f'{bag_name}_{int(source.seconds(index) * 1000):09d}'
        image_rel, label_rel = write_sample(root, split, stem, image, label, config, args.preview)
        lists[split].append(f'{image_rel} {label_rel}\n')
        print(f'\r{number + 1}/{len(indices)} {split}/{stem}', end='', flush=True)
    print()
    list_dir = root / 'lists'
    list_dir.mkdir(exist_ok=True)
    for split, lines in lists.items():
        (list_dir / f'{split}.lst').write_text(''.join(lines), encoding='utf-8')
    metadata = {
        'format': 'PIDNet list dataset',
        'num_classes': len(CLASS_NAMES),
        'ignore_label': 255,
        'classes': CLASS_NAMES,
        'source_bag': str(Path(args.bag).resolve()),
        'source_topic': args.topic,
        'interval_seconds': args.interval,
        'start_frame_index': args.start_index,
        'end_frame_index': end_index,
        'top_background_fraction': args.top_background,
        'train_samples': len(lists['train']),
        'val_samples': len(lists['val']),
    }
    with open(root / 'dataset.yaml', 'w', encoding='utf-8') as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False, allow_unicode=True)
    (root / 'classes.json').write_text(json.dumps(CLASS_NAMES, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'완료: {root} (train {len(lists["train"])}, val {len(lists["val"])})')


def parse_args():
    parser = argparse.ArgumentParser(description='bag 프레임 자동 라벨링 및 PIDNet 데이터셋 추출')
    parser.add_argument('bag', help='rosbag2 디렉터리')
    parser.add_argument('output', help='출력 데이터셋 디렉터리')
    parser.add_argument('--topic', default='/image_raw/compressed')
    parser.add_argument('--config', default=str(default_config_path()))
    parser.add_argument('--interval', type=float, default=0.5, help='프레임 간격(초), 기본 0.5')
    parser.add_argument('--start-index', type=int, default=0, help='튜너 time 바 기준 시작 프레임')
    parser.add_argument('--end-index', type=int, default=-1, help='마지막 프레임, -1이면 bag 끝')
    parser.add_argument('--count', type=int, default=0, help='추출 장수, 0이면 끝까지')
    parser.add_argument('--uniform', action='store_true', help='시작~끝 범위에서 count만큼 균등 추출')
    parser.add_argument('--top-background', type=float, default=1 / 3,
                        help='상단에서 background로 고정할 비율, 기본 1/3')
    parser.add_argument('--val-every', type=int, default=5, help='N장마다 검증셋, 0이면 전부 train')
    parser.add_argument('--no-preview', dest='preview', action='store_false')
    parser.set_defaults(preview=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit('--interval은 0보다 커야 합니다.')
    if args.count < 0:
        raise SystemExit('--count는 0 이상이어야 합니다.')
    if not 0.0 <= args.top_background <= 1.0:
        raise SystemExit('--top-background는 0~1 범위여야 합니다.')
    extract(args)


if __name__ == '__main__':
    main()
