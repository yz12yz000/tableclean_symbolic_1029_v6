"""Bounded small-cup support preserves radius and evidence rejection gates."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from rim_locator.config import RimConfig
from rim_locator.circle_fit import fit_rim_circle
from yolo_vision.tray_utils import load_tray_config, validate_tray_config, estimate_object_position, validate_objects


ROOT = Path(__file__).resolve().parents[1]


def config():
    return load_tray_config(ROOT/'src/yolo_vision/config/clear_table_live.json')


def cloud(radius, coverage=270):
    angles=np.linspace(0,np.deg2rad(coverage),400)
    return np.concatenate([np.c_[.5+radius*np.cos(angles),.2+radius*np.sin(angles),
                                np.full(len(angles),z)] for z in [.83,.84,.85]])


def test_small_cup_radius_is_allowed_in_both_geometry_paths():
    points=cloud(.033)
    with pytest.raises(ValueError):
        fit_rim_circle(points,RimConfig())
    _,fit=fit_rim_circle(points,replace(RimConfig(),circle_min_radius=.03))
    assert fit['circle_radius']==pytest.approx(.033,abs=.001)
    c=config()
    result=estimate_object_position(points,np.eye(4),c)
    assert result['position_valid'],result
    assert result['observed_rim_radius']==pytest.approx(.033,abs=.001)
    assert validate_objects([dict(result,object_id=0)],c)


def test_below_range_and_short_arc_still_rejected():
    c=config()
    for points in [cloud(.02),cloud(.033,35)]:
        assert not estimate_object_position(points,np.eye(4),c)['position_valid']


@pytest.mark.parametrize('low,high',[(.01,.15),(.03,.2),(.15,.15),(True,.15)])
def test_invalid_occupancy_radius_limits_rejected(low,high):
    c=config();c.update(visual_rim_min_radius=low,visual_rim_max_radius=high)
    with pytest.raises(ValueError):
        validate_tray_config(c)


def test_legacy_config_without_new_keys_keeps_four_cm_limit():
    c=config();c.pop('visual_rim_min_radius');c.pop('visual_rim_max_radius')
    assert validate_tray_config(c)['visual_rim_min_radius']==.04
