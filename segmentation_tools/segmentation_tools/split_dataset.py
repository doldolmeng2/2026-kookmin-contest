"""데이터셋 한 개를 담당자별 폴더로 나눈다.

프레임을 앞에서부터 잘라 나누면 담당자마다 주행의 한 구간만 맡게 되어, 한 사람의
버릇이나 실수가 트랙의 특정 구간에 몰린다. 그래서 순서대로 훑으며 비율에 맞춰
번갈아 배분한다. 모든 폴더가 원본의 전 구간과 모든 bag 을 고루 담게 된다.
"""
import argparse
import json
import shutil
from pathlib import Path

import yaml

from .core import dataset_items


def assign(count, parts):
    """프레임 순서대로 담당자를 배정한다. parts 는 (이름, 장수) 목록."""
    total = sum(size for _, size in parts)
    if total != count:
        raise ValueError(f'장수 합계가 맞지 않습니다: {total} != {count}')
    taken = {name: 0 for name, _ in parts}
    assignment = []
    for index in range(count):
        # 지금까지 받았어야 할 몫보다 가장 많이 뒤처진 담당자에게 준다.
        name = max(
            (name for name, size in parts if taken[name] < size),
            key=lambda name: dict(parts)[name] * (index + 1) / count - taken[name])
        taken[name] += 1
        assignment.append(name)
    return assignment


def split(dataset, output_root, parts, split_name='train'):
    source = Path(dataset).expanduser().resolve()
    output_root = Path(output_root).expanduser().resolve()
    items = dataset_items(source, split_name)
    assignment = assign(len(items), parts)

    metadata = {}
    metadata_path = source / 'dataset.yaml'
    if metadata_path.exists():
        metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    guide_path = source / 'ANNOTATION_GUIDE.md'

    lines = {name: [] for name, _ in parts}
    for (image_path, label_path), name in zip(items, assignment):
        output = output_root / f'{source.name}_{name}_{dict(parts)[name]}'
        for path, folder in ((image_path, 'images'), (label_path, 'labels')):
            destination = output / folder / split_name / path.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)
        preview = source / 'previews' / split_name / f'{image_path.stem}.jpg'
        if preview.exists():
            destination = output / 'previews' / split_name / preview.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(preview, destination)
        lines[name].append(
            f'images/{split_name}/{image_path.name} labels/{split_name}/{label_path.name}\n')

    outputs = []
    for name, size in parts:
        output = output_root / f'{source.name}_{name}_{size}'
        list_dir = output / 'lists'
        list_dir.mkdir(parents=True, exist_ok=True)
        (list_dir / f'{split_name}.lst').write_text(''.join(lines[name]), encoding='utf-8')
        for other in ('train', 'val'):
            if other != split_name and not (list_dir / f'{other}.lst').exists():
                (list_dir / f'{other}.lst').write_text('', encoding='utf-8')
        part_metadata = dict(metadata)
        part_metadata.update({
            'split_from': str(source),
            'assignee': name,
            f'{split_name}_samples': len(lines[name]),
        })
        (output / 'dataset.yaml').write_text(
            yaml.safe_dump(part_metadata, sort_keys=False, allow_unicode=True), encoding='utf-8')
        classes = {0: 'background', 1: 'center_lane', 2: 'left_solid',
                   3: 'right_solid', 4: 'road', 5: 'shortcut'}
        (output / 'classes.json').write_text(
            json.dumps(classes, ensure_ascii=False, indent=2), encoding='utf-8')
        if guide_path.exists():
            shutil.copy2(guide_path, output / guide_path.name)
        outputs.append((output, len(lines[name])))
        print(f'{output} ({len(lines[name])}장)')
    return outputs


def parse_part(text):
    name, _, size = text.partition('=')
    if not name or not size.isdigit():
        raise argparse.ArgumentTypeError(f'이름=장수 형식이어야 합니다: {text}')
    return name, int(size)


def main():
    parser = argparse.ArgumentParser(description='데이터셋을 담당자별 폴더로 나눈다.')
    parser.add_argument('dataset', help='나눌 데이터셋 폴더')
    parser.add_argument('output_root', help='결과 폴더들을 만들 상위 경로')
    parser.add_argument('parts', nargs='+', type=parse_part, metavar='이름=장수')
    parser.add_argument('--split', default='train')
    arguments = parser.parse_args()
    split(arguments.dataset, arguments.output_root, arguments.parts, arguments.split)


if __name__ == '__main__':
    main()
