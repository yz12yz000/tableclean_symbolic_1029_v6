#!/usr/bin/env python3
"""Replay images through the SAME single-inference adapter as the robot.

No ROS initialization, camera access, controllers or robot commands. Output
masks use original image pixels and can be inspected against aligned depth.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import cv2
import numpy as np

from yolo_vision.config import load_config, resolve_model_path
from yolo_vision.segmentation import Segmenter


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', nargs='+', type=Path)
    parser.add_argument('--vision-config', type=Path, default=root / 'src/yolo_vision/config/clear_table_vision.json')
    parser.add_argument('--model', type=Path, help='optional explicit checkpoint for a candidate profile')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    config = load_config(args.vision_config)
    if args.model:
        config = replace(config, model_path=str(args.model.resolve()))
    if not config.scene_classes:
        parser.error('single scene configuration required')
    if args.output_dir.exists():
        parser.error('output directory already exists')
    model_path = resolve_model_path(config, root / 'src/yolo_vision')
    segmenter = Segmenter(model_path, config)
    args.output_dir.mkdir(parents=True)
    reports = []
    colors = {'bowl': (30, 180, 250), 'tray': (40, 220, 40),
              'cup': (220, 80, 220), 'plate': (220, 180, 40),
              'knife': (40, 70, 240), 'fork': (240, 110, 30), 'spoon': (30, 220, 220)}
    for number, source in enumerate(args.images, 1):
        image = cv2.imread(str(source))
        if image is None or image.shape != (config.expected_height, config.expected_width, 3):
            raise ValueError('image_missing_or_dimensions_mismatch:' + str(source))
        started = time.perf_counter()
        scene = segmenter.predict_scene(image)
        elapsed = time.perf_counter()-started
        directory = args.output_dir / f'image_{number:03d}'
        directory.mkdir()
        overlay, objects, arrays = image.copy(), [], {}
        for name, items in scene.items():
            for index, item in enumerate(items):
                color = colors.get(name, (255, 150, 0))
                mask = item.mask
                arrays[f'{name}_{index}'] = mask
                overlay[mask] = (.65*overlay[mask] + .35*np.asarray(color)).astype(np.uint8)
                contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, color, 2)
                yy, xx = np.nonzero(mask)
                x, y = int(xx.min()), int(yy.min())
                cv2.putText(overlay, f'{name} #{index} {item.confidence:.3f}', (x, max(16, y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, color, 2)
                objects.append(dict(class_name=name, index=index, confidence=item.confidence,
                                    bbox_xyxy=[x, y, int(xx.max())+1, int(yy.max())+1],
                                    mask_pixels=int(mask.sum())))
        np.savez_compressed(directory / 'masks_original.npz', **arrays)
        cv2.imwrite(str(directory / 'overlay.png'), overlay)
        x0, y0, x1, y1 = config.inference_roi
        cv2.imwrite(str(directory / 'table_2x.png'), cv2.resize(overlay[y0:y1, x0:x1], None, fx=2, fy=2))
        report = dict(source=str(source.resolve()), model=str(model_path),
                      roi=config.inference_roi, imgsz=config.imgsz, confidence=config.confidence,
                      model_inferences=1, wall_seconds=elapsed,
                      cold_start=(number == 1), hardware_motion=False,
                      counts={name: len(items) for name, items in scene.items()}, objects=objects)
        (directory / 'scene.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        reports.append(report)
        print(str(source), report['counts'], f'{elapsed:.3f}s', flush=True)
    (args.output_dir / 'results.json').write_text(json.dumps(reports, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
