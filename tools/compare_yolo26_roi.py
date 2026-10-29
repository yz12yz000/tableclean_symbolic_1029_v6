"""Compare all segmentation classes in the user-specified tabletop ROI."""
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.engine.results import Results

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/yolo26_roi_comparison_20261019'
ROI = (340, 380, 930, 610)


def save_image(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(str(path))


def main():
    OUT.mkdir(exist_ok=True)
    config_paths = sorted((ROOT / 'src/yolo_vision/config').glob('*.json'))
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in config_paths}
    rows = []
    x0, y0, x1, y1 = ROI
    settings = dict(conf=0.25, imgsz=512, device='intel:cpu',
                    retina_masks=True, rect=False, verbose=False, save=False)
    inputs = {}
    for index, scale in enumerate('nsmlx', 1):
        name = f'yolo26{scale}-seg'
        model_path = ROOT / f'src/yolo_vision/models/yolo26_backups/{name}_openvino_model'
        model = YOLO(str(model_path), task='segment')
        for source in sorted((ROOT / 'image').glob('*.png')):
            image = cv2.imread(str(source))
            assert image is not None and image.shape[:2] == (800, 1280)
            inputs[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
            crop = np.ascontiguousarray(image[y0:y1, x0:x1])
            result = model.predict(source=crop, **settings)[0]
            boxes = result.boxes.data.cpu().clone()
            boxes[:, [0, 2]] += x0
            boxes[:, [1, 3]] += y0
            full_masks = None
            if result.masks is not None:
                masks = result.masks.data.cpu()
                assert tuple(masks.shape[1:]) == (230, 590)
                full_masks = torch.zeros((len(masks), 800, 1280), dtype=masks.dtype)
                full_masks[:, y0:y1, x0:x1] = masks
            mapped = Results(orig_img=image, path=str(source), names=result.names,
                             boxes=boxes, masks=full_masks)
            overlay = mapped.plot(line_width=1, font_size=10)
            filename = f'{source.stem}_{index}_YOLO26{scale.upper()}'
            save_image(OUT / 'full_frame' / f'{filename}.jpg', overlay)
            zoom = cv2.resize(overlay[y0:y1, x0:x1], (1180, 460), interpolation=cv2.INTER_LINEAR)
            save_image(OUT / 'comparison_2x' / f'{filename}.jpg', zoom)
            data_dir = OUT / 'data' / name / source.stem
            data_dir.mkdir(parents=True, exist_ok=True)
            (data_dir / 'detections_full_frame.json').write_text(mapped.to_json(), encoding='utf-8')
            if full_masks is not None:
                np.savez_compressed(data_dir / 'masks_full_frame.npz', masks=full_masks.numpy().astype(bool))
            counts = {}
            for box in result.boxes:
                label = result.names[int(box.cls.item())]
                counts[label] = counts.get(label, 0) + 1
            row = dict(model=name, image=source.name, detections=len(boxes),
                       bowls=counts.get('bowl', 0), classes=json.dumps(counts),
                       inference_ms=result.speed['inference'])
            rows.append(row)
            print(json.dumps(row), flush=True)
        del model
    with (OUT / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    manifest = dict(roi=ROI, display_roi=ROI, crop_size=[590, 230], display_size=[1180, 460],
                    settings=settings, preprocessing='Ultralytics centered letterbox to 512x512; no tiling',
                    inputs=inputs, results=rows, config_hashes=before,
                    current_clear_table_config=json.loads((ROOT / 'src/yolo_vision/config/clear_table_vision.json').read_text()))
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
    assert before == {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in config_paths}


if __name__ == '__main__':
    main()
