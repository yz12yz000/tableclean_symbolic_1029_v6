"""Local tray model contract; no weights, hardware or network are required."""

import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from yolo_vision.config import resolve_model_path
from yolo_vision.segmentation import Segmenter, TraySegmentationConfig, select_targets


def config(**changes):
    return TraySegmentationConfig(**{
        'model_path': 'models/tray-seg.pt', 'target_class': 'tray',
        'confidence': .5, 'imgsz': 640, 'device': 'cpu', **changes})


def install_model(monkeypatch, *, task='segment', names=None):
    calls = []
    model = NS(task=task, names={0: 'tray'} if names is None else names)

    def load(path, **kwargs):
        calls.append((path, kwargs))
        return model

    monkeypatch.setitem(sys.modules, 'ultralytics', NS(YOLO=load))
    return model, calls


def test_absent_tray_weights_fail_without_yolo_download(monkeypatch, tmp_path):
    _, calls = install_model(monkeypatch)
    with pytest.raises(ValueError, match='segmentation_model_missing'):
        Segmenter(tmp_path / 'tray-seg.pt', config(), require_target=True)
    assert not calls


@pytest.mark.parametrize('task,names,error', [
    ('detect', {0: 'tray'}, 'requires_segmentation'),
    ('segment', {0: 'bowl', 1: 'dining table'}, 'missing_target_class:tray'),
    ('segment', None, None),
    ('segment', ['bowl', 'tray'], None),
])
def test_pt_checkpoint_must_contain_real_tray_segmentation(monkeypatch, tmp_path, task, names, error):
    path = tmp_path / 'custom.pt'
    path.touch()
    model, calls = install_model(monkeypatch, task=task, names=names)
    if error:
        with pytest.raises(ValueError, match=error):
            Segmenter(path, config(), require_target=True)
    else:
        assert Segmenter(path, config(), require_target=True).model is model
    assert calls == [(str(path), {'task': 'segment'})]


@pytest.mark.parametrize('metadata,error', [
    (None, 'metadata_missing'),
    ('[]', 'metadata_invalid'),
    ('task: detect\nnames: {0: tray}', 'requires_segmentation'),
    ('task: segment\nnames: {0: bowl}', 'missing_target_class:tray'),
    ('task: segment\nnames: {0: tray}', None),
])
def test_openvino_export_metadata_is_validated_before_loading(monkeypatch, tmp_path, metadata, error):
    (tmp_path / 'model.xml').touch()
    (tmp_path / 'model.bin').touch()
    if metadata is not None:
        (tmp_path / 'metadata.yaml').write_text(metadata)
    model, calls = install_model(monkeypatch)
    if error:
        with pytest.raises(ValueError, match=error):
            Segmenter(tmp_path, config(), require_target=True)
        assert not calls
    else:
        assert Segmenter(tmp_path, config(), require_target=True).model is model


def test_tray_model_uses_one_full_frame_without_bowl_roi_or_tiles():
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = config()
    observed = []
    segmenter._predict_image = lambda bgr: observed.append(bgr) or []
    bgr = np.zeros((100, 200, 3), dtype=np.uint8)
    assert segmenter.predict_all(bgr) == []
    assert len(observed) == 1 and observed[0] is bgr


def test_relative_tray_model_path_is_package_share_relative(tmp_path):
    assert resolve_model_path(config(), tmp_path) == tmp_path / 'models/tray-seg.pt'
    absolute = tmp_path / 'other/custom.pt'
    assert resolve_model_path(config(model_path=str(absolute)), '/unused') == absolute


def test_a_tray_box_without_segmentation_mask_is_rejected():
    def tensor(values):
        return NS(cpu=lambda: NS(numpy=lambda: np.asarray(values)))

    result = NS(boxes=NS(cls=tensor([0]), conf=tensor([.9])), names={0: 'tray'}, masks=None)
    with pytest.raises(ValueError, match='missing_target_masks'):
        select_targets(result, 'tray', (100, 200, 3))


def test_empty_tray_detections_are_observable_not_a_fake_tray():
    assert select_targets(NS(boxes=None), 'tray', (100, 200, 3)) == []
