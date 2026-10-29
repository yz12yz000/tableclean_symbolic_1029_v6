"""Reject mislabelled or stale reference coordinates before loading YOLOE."""
from pathlib import Path
import json

import pytest
from build_table_visual_prompt import load_profile


ROOT = Path(__file__).resolve().parents[1]


def test_committed_profiles_have_valid_class_mapping_and_original_coordinates():
    library = ROOT / 'visual_prompt_library'
    for path in sorted((library / 'profiles').glob('*.json')):
        data = load_profile(library, path)
        assert set(data['ids']) == set(range(len(data['names'])))
        assert data['image_size'] == [1280, 800]
        assert len(data['profile']['inference_roi']) == 4


def test_seven_class_profile_keeps_existing_bowl_tray_ids():
    library = ROOT / 'visual_prompt_library'
    data = load_profile(library, library/'profiles/live_table_20261022.json')
    assert data['names'] == ['bowl', 'tray', 'cup', 'plate', 'knife', 'fork', 'spoon']
    assert data['ids'] == [0, 0, 0, 1, 2, 3, 4, 5, 6]


@pytest.mark.parametrize('mutation,error', [
    ('polygon_outside', 'invalid_prompt_polygon'),
    ('polygon_nan', 'invalid_prompt_polygon'),
    ('exclude_self', 'invalid_prompt_exclusions'),
    ('missing_spoon', 'reference_missing_classes:spoon'),
])
def test_seven_class_contour_references_are_validated(tmp_path, mutation, error):
    library = ROOT / 'visual_prompt_library'
    classes = json.loads((library/'classes.json').read_text())
    profile = json.loads((library/'profiles/live_table_20261022.json').read_text())
    annotation = json.loads((library/'annotations/live_table_contours_20261022.json').read_text())
    annotation['image'] = 'image.png'
    profile['annotation'] = 'annotation.json'
    (tmp_path/'image.png').touch()
    if mutation == 'polygon_outside':
        annotation['objects'][0]['polygon_xy'][0] = [0, 0]
    elif mutation == 'polygon_nan':
        annotation['objects'][0]['polygon_xy'][0] = [float('nan'), 450]
    elif mutation == 'exclude_self':
        annotation['objects'][0]['exclude_class_ids'] = [0]
    else:
        annotation['objects'] = [o for o in annotation['objects'] if o['class_id'] != 6]
    for name, value in [('classes.json', classes), ('profile.json', profile), ('annotation.json', annotation)]:
        (tmp_path/name).write_text(json.dumps(value))
    with pytest.raises(ValueError, match=error):
        load_profile(tmp_path, tmp_path/'profile.json')


@pytest.mark.parametrize('mutation,error', [
    ('missing_class', 'requires_bowl_and_tray'),
    ('outside_box', 'outside_reference_roi'),
    ('bad_class', 'invalid_prompt_class'),
    ('class_order', 'stable_bowl_0_tray_1'),
    ('bad_roi', 'invalid_reference_roi'),
    ('outside_path', 'outside_library'),
])
def test_invalid_library_rejected(tmp_path, mutation, error):
    library = ROOT / 'visual_prompt_library'
    classes = json.loads((library / 'classes.json').read_text())
    profile = json.loads((library / 'profiles/pattern_table.json').read_text())
    annotation = json.loads((library / 'annotations/pattern_table.json').read_text())
    profile['annotation'] = 'annotation.json'
    annotation['image'] = 'reference.png'
    (tmp_path / 'reference.png').touch()
    if mutation == 'missing_class':
        annotation['objects'] = [o for o in annotation['objects'] if o['class_id'] == 0]
    elif mutation == 'outside_box':
        annotation['objects'][0]['bbox_xyxy'] = [0, 0, 30, 30]
    elif mutation == 'bad_class':
        annotation['objects'][0]['class_id'] = True
    elif mutation == 'class_order':
        classes['classes'].reverse()
    elif mutation == 'bad_roi':
        profile['inference_roi'] = [0, 0, 5000, 800]
    else:
        annotation['image'] = '../outside.png'
    for name, value in [('classes.json', classes), ('profile.json', profile), ('annotation.json', annotation)]:
        (tmp_path / name).write_text(json.dumps(value))
    with pytest.raises(ValueError, match=error):
        load_profile(tmp_path, tmp_path / 'profile.json')
