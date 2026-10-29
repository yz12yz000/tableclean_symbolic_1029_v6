"""YOLO/OpenVINO adapter. NumPy inputs are BGR, as required by Ultralytics."""

from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class SegmentationResult:
    mask: np.ndarray
    class_name: str
    confidence: float


@dataclass(frozen=True)
class TraySegmentationConfig:
    """Independent tray model; always infer the complete synchronized image."""

    model_path: str
    target_class: str
    confidence: float
    imgsz: int
    device: str
    inference_roi: None = None
    inference_tile_size: int = 0


def _validate_target_metadata(task, names, target_class):
    if task != "segment":
        raise ValueError("tray_yolo_model_requires_segmentation")
    names = list(names.values()) if isinstance(names, dict) else names
    if not isinstance(names, (list, tuple)) or target_class not in names:
        raise ValueError("tray_yolo_model_missing_target_class:" + target_class)


def _mask_for(result, index, image_shape):
    mask = result.masks.data[index].cpu().numpy()
    if mask.shape != tuple(image_shape[:2]):
        raise ValueError("dimension_mismatch:mask_not_in_original_image_coordinates")
    mask = mask > 0.5
    if not mask.any():
        raise ValueError("invalid_inference_result:empty_target_mask")
    return mask


def select_targets(result, target_class, image_shape):
    """Return every finite-confidence instance of ``target_class``.

    The detector order is made deterministic by confidence (descending) and
    the original model index.  Object IDs are local to one synchronized frame.
    """
    if result.boxes is None:
        return []
    classes = result.boxes.cls.cpu().numpy().astype(int)
    scores = result.boxes.conf.cpu().numpy()
    matches = [i for i, cls in enumerate(classes)
               if result.names[int(cls)] == target_class and np.isfinite(scores[i])]
    if not matches:
        return []
    if result.masks is None:
        raise ValueError("invalid_inference_result:missing_target_masks")
    targets = []
    for index in sorted(matches, key=lambda i: (-float(scores[i]), i)):
        targets.append(SegmentationResult(_mask_for(result, index, image_shape),
                                           target_class, float(scores[index])))
    return targets


def suppress_scene_duplicates(scene, threshold=.85):
    """Resolve near-identical masks labelled as multiple classes/instances.

    This is deliberately stricter than ordinary box NMS. A spoon inside a bowl
    or a bowl inside a tray has small mask IoU and remains a separate instance.
    Only almost the same pixel region loses its lower-confidence duplicate.
    """
    ordered = sorted((item for items in scene.values() for item in items),
                     key=lambda item: -item.confidence)
    kept = []
    for item in ordered:
        if any(np.count_nonzero(item.mask & other.mask) /
               max(1, np.count_nonzero(item.mask | other.mask)) >= threshold for other in kept):
            continue
        kept.append(item)
    return {name: [item for item in kept if item.class_name == name] for name in scene}



class Segmenter:
    def __init__(self, model_path, config, *, require_target=False):
        path = Path(model_path)
        # Reject absent local weights before YOLO can interpret the name as a
        # downloadable asset. The original bowl OpenVINO path remains valid.
        if not path.exists():
            raise ValueError(f"segmentation_model_missing:{path}")
        is_openvino = path.is_dir()
        if is_openvino and (not list(path.glob("*.xml")) or not list(path.glob("*.bin"))):
            raise ValueError(f"openvino_model_missing:{path}")
        if not is_openvino and path.suffix.lower() != ".pt":
            raise ValueError(f"segmentation_model_format_not_supported:{path}")
        if require_target and is_openvino:
            # An exported backend does not infer its task from constructor
            # arguments. Validate the export metadata rather than force a
            # detection model to masquerade as a segmentation model.
            import yaml

            metadata_path = path / "metadata.yaml"
            if not metadata_path.is_file():
                raise ValueError("tray_yolo_model_metadata_missing")
            with metadata_path.open(encoding="utf-8") as stream:
                metadata = yaml.safe_load(stream)
            if not isinstance(metadata, dict):
                raise ValueError("tray_yolo_model_metadata_invalid")
            _validate_target_metadata(metadata.get("task"), metadata.get("names"), config.target_class)
        self.config = config
        scene_classes = getattr(config, 'scene_classes', None)
        # Frozen YOLOE checkpoints run through the standard segmentation
        # predictor. YOLOE.predict would reset class-agnostic NMS on every call;
        # overlapping bowl/tray instances must remain independent classes.
        from ultralytics import YOLO
        self.model = YOLO(str(path), task="segment")
        if scene_classes is not None:
            # No text encoding or prompt changes on the robot: checkpoint/export
            # must already contain every required visual-prompt class.
            for name in scene_classes:
                _validate_target_metadata(self.model.task, self.model.names, name)
        elif require_target and not is_openvino:
            # For .pt, Ultralytics reads the checkpoint's actual task/classes.
            _validate_target_metadata(self.model.task, self.model.names, config.target_class)

    def predict_scene(self, bgr):
        """One cropped inference, independent class masks in ORIGINAL pixels.

        The bowl and tray masks can overlap. They must never compete for pixels
        here: bowl occluders are handled by the existing tray geometry routine.
        """
        classes = getattr(self.config, 'scene_classes', None)
        if not classes or self.config.inference_tile_size:
            raise ValueError('scene_inference_requires_classes_and_no_tiles')
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError('invalid_bgr_image')
        roi = self.config.inference_roi
        if roi is None:
            raise ValueError('scene_inference_requires_table_roi')
        x0, y0, x1, y1 = roi
        if x1 > bgr.shape[1] or y1 > bgr.shape[0]:
            raise ValueError('inference_roi_outside_image')
        image = np.ascontiguousarray(bgr[y0:y1, x0:x1])
        predictions = self.model.predict(
            source=image, conf=min([self.config.confidence, *self.config.class_confidences.values()]), imgsz=self.config.imgsz,
            device=self.config.device, retina_masks=True, rect=False,
            agnostic_nms=False, verbose=False, save=False)
        if len(predictions) != 1:
            raise ValueError('invalid_inference_result')
        scene = {}
        for name in classes:
            output_name = self.config.class_aliases.get(name, name)
            restored = scene.setdefault(output_name, [])
            for target in select_targets(predictions[0], name, image.shape):
                if target.confidence < self.config.class_confidences.get(name, self.config.confidence):
                    continue
                mask = np.zeros(bgr.shape[:2], dtype=bool)
                mask[y0:y1, x0:x1] = target.mask
                restored.append(SegmentationResult(mask, output_name, target.confidence))
        return suppress_scene_duplicates(scene)

    def predict_all(self, bgr):
        if getattr(self.config, 'scene_classes', None) is not None:
            return self.predict_scene(bgr)[self.config.target_class]
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("invalid_bgr_image")
        roi = self.config.inference_roi
        image = bgr
        if roi is not None:
            x0, y0, x1, y1 = roi
            if x1 > bgr.shape[1] or y1 > bgr.shape[0]:
                raise ValueError("inference_roi_outside_image")
            image = np.ascontiguousarray(bgr[y0:y1, x0:x1])
        selected = self._predict_tiles(image) if self.config.inference_tile_size else self._predict_image(image)
        if roi is None:
            return selected
        # Restore ORIGINAL pixel coordinates before selecting organized XYZ.
        restored = []
        for target in selected:
            mask = np.zeros(bgr.shape[:2], dtype=bool)
            mask[y0:y1, x0:x1] = target.mask
            restored.append(SegmentationResult(mask, target.class_name, target.confidence))
        return restored

    def _predict_image(self, image):
        results = self.model.predict(
            source=image, conf=self.config.confidence, imgsz=self.config.imgsz,
            device=self.config.device, retina_masks=True, rect=False,
            verbose=False, save=False,
        )
        if len(results) != 1:
            raise ValueError("invalid_inference_result")
        return select_targets(results[0], self.config.target_class, image.shape)

    def _predict_tiles(self, image):
        height, width = image.shape[:2]
        # Keep a whole-view pass for large objects that cannot fit inside a tile.
        selected = list(self._predict_image(image))
        views = [0] * len(selected)
        windows = tile_windows(width, height, self.config.inference_tile_size, self.config.tile_overlap)
        # Crop the SAME synchronized image; never combine detections from
        # different acquisition times. Overlapping windows cover the full view.
        for view, (x0, y0, x1, y1) in enumerate(windows, 1):
            crop = np.ascontiguousarray(image[y0:y1, x0:x1])
            for target in self._predict_image(crop):
                yy, xx = np.nonzero(target.mask)
                # Internal crop boundaries can cut a bowl in half and yield a
                # misleadingly high score. A neighboring window supplies its
                # full mask. Actual image edges are not artificial tile seams.
                if ((x0 > 0 and xx.min() < 2) or (y0 > 0 and yy.min() < 2)
                        or (x1 < width and xx.max() >= x1-x0-2)
                        or (y1 < height and yy.max() >= y1-y0-2)):
                    continue
                mask = np.zeros((height, width), dtype=bool)
                mask[y0:y1, x0:x1] = target.mask
                selected.append(SegmentationResult(mask, target.class_name, target.confidence))
                views.append(view)
        return suppress_duplicate_masks(selected, self.config.tile_mask_iou, views=views,
                                        min_support=2, single_confidence=self.config.tile_single_confidence)


def tile_windows(width, height, size, overlap):
    def starts(length):
        if length <= size:
            return [0]
        stride = max(1, round(size*(1-overlap)))
        return sorted(set(list(range(0, length-size+1, stride)) + [length-size]))
    return [(x, y, min(x+size, width), min(y+size, height)) for y in starts(height) for x in starts(width)]


def suppress_duplicate_masks(targets, threshold, views=None, min_support=1, single_confidence=0.):
    groups = []
    views = list(range(len(targets))) if views is None else views
    for index in sorted(range(len(targets)), key=lambda i: -targets[i].confidence):
        target = targets[index]
        for other, supporting_views in groups:
            overlap = np.count_nonzero(target.mask & other.mask) / max(1, np.count_nonzero(target.mask | other.mask))
            if overlap >= threshold:
                supporting_views.add(views[index])
                break
        else:
            groups.append((target, {views[index]}))
    return [target for target, supporting_views in groups
            if target.confidence >= single_confidence or len(supporting_views) >= min_support]
