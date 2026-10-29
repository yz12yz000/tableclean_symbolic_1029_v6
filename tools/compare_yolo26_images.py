"""Offline comparison of YOLO26 segmentation scales; never edits live config."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import ultralytics
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scales', default='n,s,m,l,x')
    parser.add_argument('--existing-default', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config_path = ROOT / 'src/yolo_vision/config/config.json'
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    backup = ROOT / 'src/yolo_vision/models/yolo26_backups'
    backup.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    images = sorted((ROOT / 'image').glob('*.png'))
    settings = dict(conf=config['confidence'], imgsz=config['imgsz'],
                    device=config['device'], retina_masks=True, rect=False,
                    verbose=False, save=False)
    manifest = dict(config=config, predict_settings=settings,
                    ultralytics_version=ultralytics.__version__,
                    inputs={p.name: sha256(p) for p in images}, models=[])
    rows = []
    for scale in args.scales.split(','):
        name = f'yolo26{scale}-seg'
        if args.existing_default:
            if scale != 'n':
                raise ValueError('Existing default supports n only')
            exported = ROOT / 'src/yolo_vision' / config['model_path']
            source = {'existing_default': str(exported)}
        else:
            weights = backup / f'{name}.pt'
            if not weights.is_file():
                raise FileNotFoundError(f'Download official weights first: {weights}')
            exported = backup / f'{name}_openvino_model'
            if not (exported / f'{name}.xml').is_file():
                YOLO(str(weights), task='segment').export(
                    format='openvino', imgsz=config['imgsz'], batch=1,
                    dynamic=False, half=False, int8=False, device='cpu',
                    end2end=False)
            source = dict(weights=str(weights), sha256=sha256(weights),
                          url=f'https://github.com/ultralytics/assets/releases/download/v8.4.0/{name}.pt')
        manifest['models'].append(dict(name=name, source=source, openvino=str(exported)))
        model = YOLO(str(exported), task='segment')
        dest = args.output / name
        dest.mkdir(exist_ok=True)
        for path in images:
            result = model.predict(source=str(path), **settings)[0]
            result.save(filename=str(dest / f'{path.stem}_all.jpg'))
            selected = [i for i, box in enumerate(result.boxes)
                        if result.names[int(box.cls.item())] == config['target_class']]
            bowls = result[selected]
            bowls.save(filename=str(dest / f'{path.stem}_bowl.jpg'))
            (dest / f'{path.stem}.json').write_text(result.to_json(), encoding='utf-8')
            masks = dest / f'{path.stem}_masks'
            masks.mkdir(exist_ok=True)
            if result.masks is not None:
                for i, mask in enumerate(result.masks.data.cpu().numpy()):
                    if not cv2.imwrite(str(masks / f'{i:03d}.png'), (mask > 0.5).astype(np.uint8) * 255):
                        raise IOError('Failed to save mask')
            row = dict(model=name, image=path.name, detections=len(result.boxes),
                       bowls=len(selected), inference_ms=result.speed['inference'])
            rows.append(row)
            print(json.dumps(row), flush=True)
        del model
    manifest['results'] = rows
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    with (args.output / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    assert config_path.read_bytes() == config_bytes, 'Default config changed during comparison'


if __name__ == '__main__':
    main()
