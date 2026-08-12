import argparse
import json
import shutil
from pathlib import Path

import yaml


def merge(inputs, output):
    output = Path(output).expanduser().resolve()
    lines = []
    sources = []
    for input_path in inputs:
        source = Path(input_path).expanduser().resolve()
        metadata_path = source / 'dataset.yaml'
        if metadata_path.exists():
            metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
            sources.append(metadata.get('source_bag', str(source)))
        list_path = source / 'lists' / 'train.lst'
        for line in list_path.read_text(encoding='utf-8').splitlines():
            if not line.strip():
                continue
            image_rel, label_rel = line.split()[:2]
            for relative in (image_rel, label_rel):
                destination = output / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source / relative, destination)
            preview_rel = Path('previews/train') / Path(image_rel).with_suffix('.jpg').name
            if (source / preview_rel).exists():
                destination = output / preview_rel
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source / preview_rel, destination)
            lines.append(f'{image_rel} {label_rel}\n')
    list_dir = output / 'lists'
    list_dir.mkdir(parents=True, exist_ok=True)
    (list_dir / 'train.lst').write_text(''.join(lines), encoding='utf-8')
    (list_dir / 'val.lst').write_text('', encoding='utf-8')
    metadata = {
        'format': 'PIDNet list dataset', 'num_classes': 6, 'ignore_label': 255,
        'source_bags': sources, 'train_samples': len(lines), 'val_samples': 0,
    }
    (output / 'dataset.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True), encoding='utf-8')
    classes = {0: 'background', 1: 'center_lane', 2: 'left_solid',
               3: 'right_solid', 4: 'road', 5: 'shortcut'}
    (output / 'classes.json').write_text(
        json.dumps(classes, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'병합 완료: {output} ({len(lines)}장)')


def main():
    parser = argparse.ArgumentParser(description='추출된 데이터셋 여러 개를 한 검수 묶음으로 병합')
    parser.add_argument('output')
    parser.add_argument('inputs', nargs='+')
    args = parser.parse_args()
    merge(args.inputs, args.output)


if __name__ == '__main__':
    main()
