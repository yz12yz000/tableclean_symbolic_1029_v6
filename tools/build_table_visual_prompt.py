#!/usr/bin/env python3
"""Build a frozen visual-prompt checkpoint, offline and without ROS.

One reference scene per profile keeps sample selection explicit. Boxes or
polygons of a class use YOLOE's native visual prompt extraction. No custom
embedding averaging, text encoder, training or runtime reference image is used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def library_path(root, value):
    path = (root / value).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('reference_outside_library')
    if not path.is_file():
        raise ValueError('reference_missing:' + str(path))
    return path


def load_profile(library, profile_path):
    """Validate original-pixel annotations before loading any neural model."""
    library = Path(library).resolve()
    profile = read_json(profile_path)
    classes = read_json(library / 'classes.json')
    entries = classes.get('classes', []) if isinstance(classes, dict) else []
    if (not isinstance(classes, dict) or type(classes.get('schema_version')) is not int
            or classes['schema_version'] != 1 or not isinstance(entries, list)
            or entries[:2] != [{'id': 0, 'name': 'bowl'}, {'id': 1, 'name': 'tray'}]):
        raise ValueError('table_library_requires_stable_bowl_0_tray_1')
    for index, entry in enumerate(entries):
        if (not isinstance(entry, dict) or type(entry.get('id')) is not int or entry['id'] != index
                or not isinstance(entry.get('name'), str) or not entry['name'].strip()
                or entry['name'] != entry['name'].strip()):
            raise ValueError('invalid_library_class_mapping')
    all_names = [entry['name'] for entry in entries]
    if len(set(all_names)) != len(all_names):
        raise ValueError('duplicate_library_class_name')
    names = profile.get('classes', ['bowl', 'tray'])
    if (not isinstance(names, list) or names[:2] != ['bowl', 'tray']
            or any(not isinstance(n, str) or n not in all_names for n in names)
            or len(set(names)) != len(names)
            or names != [n for n in all_names if n in names]):
        raise ValueError('invalid_profile_classes')
    selected_ids = {all_names.index(name): index for index, name in enumerate(names)}
    if profile.get('schema_version') != 1:
        raise ValueError('unsupported_profile_schema')
    annotation_path = library_path(library, profile['annotation'])
    annotation = read_json(annotation_path)
    if annotation.get('schema_version') != 1:
        raise ValueError('unsupported_annotation_schema')
    image = library_path(library, annotation['image'])
    size = annotation['image_size']
    if (not isinstance(size, list) or len(size) != 2
            or any(type(v) is not int or v <= 0 for v in size)):
        raise ValueError('invalid_reference_image_size')
    roi = profile['inference_roi']
    if (not isinstance(roi, list) or len(roi) != 4 or any(type(v) is not int for v in roi)
            or not 0 <= roi[0] < roi[2] <= size[0] or not 0 <= roi[1] < roi[3] <= size[1]):
        raise ValueError('invalid_reference_roi')
    imgsz = profile['imgsz']
    if type(imgsz) is not int or imgsz < 32 or imgsz % 32:
        raise ValueError('invalid_prompt_imgsz')
    boxes, ids = [], []
    for obj in annotation['objects']:
        cid, box = obj['class_id'], obj['bbox_xyxy']
        if type(cid) is not int or cid not in selected_ids:
            raise ValueError('invalid_prompt_class')
        if (not isinstance(box, list) or len(box) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in box)
                or not roi[0] <= box[0] < box[2] <= roi[2]
                or not roi[1] <= box[1] < box[3] <= roi[3]):
            raise ValueError('prompt_box_outside_reference_roi')
        boxes.append([box[0]-roi[0], box[1]-roi[1], box[2]-roi[0], box[3]-roi[1]])
        ids.append(selected_ids[cid])
        polygon = obj.get('polygon_xy')
        if polygon is not None:
            if (not isinstance(polygon, list) or len(polygon) < 3
                    or any(not isinstance(p, list) or len(p) != 2
                           or any(type(v) not in (int, float) or not math.isfinite(v) for v in p)
                           or not roi[0] <= p[0] < roi[2] or not roi[1] <= p[1] < roi[3]
                           for p in polygon)):
                raise ValueError('invalid_prompt_polygon')
        excludes = obj.get('exclude_class_ids', [])
        if (not isinstance(excludes, list)
                or any(type(i) is not int or i not in selected_ids or i == cid for i in excludes)):
            raise ValueError('invalid_prompt_exclusions')
    if not {0, 1} <= set(ids):
        raise ValueError('reference_requires_bowl_and_tray')
    if set(ids) != set(range(len(names))):
        raise ValueError('reference_missing_classes:' + ','.join(names[i] for i in range(len(names)) if i not in ids))
    return dict(profile=profile, classes=classes, image=image, image_size=size,
                annotation_path=annotation_path, annotation=annotation, boxes=boxes, ids=ids,
                names=names)


def build(library, profile_path, base_model, output):
    data = load_profile(library, profile_path)
    base_model, output = Path(base_model).resolve(), Path(output).resolve()
    if not base_model.is_file() or base_model.suffix != '.pt':
        raise ValueError('local_yoloe_base_checkpoint_required')
    if output.exists():
        raise ValueError('release_already_exists')
    import cv2
    import numpy as np
    import torch
    import ultralytics
    from ultralytics import YOLO, YOLOE
    from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor

    torch.set_num_threads(4)
    image = cv2.imread(str(data['image']))
    if image is None or list(image.shape[1::-1]) != data['image_size']:
        raise ValueError('reference_image_dimensions_mismatch')
    x0, y0, x1, y1 = data['profile']['inference_roi']
    reference = np.ascontiguousarray(image[y0:y1, x0:x1])
    model = YOLOE(str(base_model), task='segment')
    names = data['names']
    if any('polygon_xy' in obj for obj in data['annotation']['objects']):
        # Precise references avoid teaching a plate's appearance as knife/fork,
        # or teaching the spoon as part of the bowl that contains it.
        from ultralytics.data.augment import LetterBox, LoadVisualPrompt
        masks = []
        for obj, box in zip(data['annotation']['objects'], data['boxes']):
            mask = np.zeros(reference.shape[:2], np.uint8)
            polygon = obj.get('polygon_xy')
            if polygon is not None:
                points = np.asarray(polygon, np.float32)-[x0, y0]
                cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
            else:
                ax, ay, bx, by = map(round, box)
                mask[ay:by, ax:bx] = 1
            if not mask.any():
                raise ValueError('empty_reference_polygon')
            masks.append(mask)
        originals = [m.copy() for m in masks]
        for index, obj in enumerate(data['annotation']['objects']):
            for other, other_mask in zip(data['annotation']['objects'], originals):
                if other['class_id'] in obj.get('exclude_class_ids', []):
                    masks[index][other_mask.astype(bool)] = 0
            if not masks[index].any():
                raise ValueError('empty_reference_after_exclusions')
        size = data['profile']['imgsz']
        rgb = LetterBox((size, size), auto=False)(image=reference)
        resize_mask = LetterBox((size, size), auto=False, padding_value=0,
                               interpolation=cv2.INTER_NEAREST)
        resized = np.stack([resize_mask(image=m[..., None])[..., 0] for m in masks])
        visuals = LoadVisualPrompt().get_visuals(np.asarray(data['ids']), (size, size), masks=resized)
        image_tensor = torch.from_numpy(np.ascontiguousarray(rgb[..., ::-1].transpose(2, 0, 1))).float()[None]/255.
        model.model.eval()
        model.model.model[-1].nc = len(names)
        embeddings = model.get_visual_pe(image_tensor, visuals[None])
        model.model.set_classes(names, embeddings.detach().clone())
    else:
        model.predict(source=reference, refer_image=reference,
                      visual_prompts={'bboxes': np.asarray(data['boxes'], dtype=np.float32),
                                      'cls': np.asarray(data['ids'], dtype=np.int64)},
                      predictor=YOLOEVPSegPredictor, device='cpu', imgsz=data['profile']['imgsz'],
                      rect=False, conf=.25, retina_masks=True, verbose=False, save=False)
        model.model.set_classes(names, model.model.pe.detach().clone())
    model.predictor = None
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.prompt-build-', dir=output.parent) as temporary:
        stage = Path(temporary) / 'release'
        stage.mkdir()
        model.save(str(stage / 'model.pt'))
        # Test the exact loader used on the robot, no prompts or text encoder.
        restored = YOLO(str(stage / 'model.pt'), task='segment')
        if restored.task != 'segment' or list(restored.names.values()) != names:
            raise ValueError('saved_prompt_class_mismatch')
        prediction = restored.predict(source=reference, device='cpu', imgsz=data['profile']['imgsz'],
                                      conf=.25, retina_masks=True, rect=False, agnostic_nms=False,
                                      verbose=False, save=False)[0]
        cv2.imwrite(str(stage / 'reference_overlay.jpg'), prediction.plot())
        counts = {name: 0 for name in names}
        for cls in prediction.boxes.cls.cpu().tolist():
            counts[names[int(cls)]] += 1
        manifest = dict(schema_version=1, method='yoloe_visual_prompt_no_training',
                        classes=names, profile=data['profile'], annotation=data['annotation'],
                        ultralytics_version=ultralytics.__version__,
                        base_model_sha256=digest(base_model), reference_sha256=digest(data['image']),
                        annotation_sha256=digest(data['annotation_path']),
                        model_sha256=digest(stage / 'model.pt'),
                        validation_status='reference_smoke_only_not_hardware_approved')
        (stage / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
        (stage / 'validation.json').write_text(json.dumps(dict(
            reference_only=True, detected_counts=counts, hardware_motion=False), indent=2)+'\n')
        stage.rename(output)
    return output


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=root / 'visual_prompt_library')
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--base-model', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    try:
        print(build(args.library, args.profile, args.base_model, args.output_dir))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
