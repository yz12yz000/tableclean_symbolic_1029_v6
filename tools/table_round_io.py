"""Immutable visual request bundles and atomic progress pointers."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
from table_round_protocol import request_revision, validate_request


def atomic_json(path, value, *, replace=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        if replace:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)  # atomic, never overwrites a competing submission
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def render_overlay(directory, vision, options):
    import cv2
    import numpy as np
    directory = Path(directory)
    raw = cv2.imread(str(directory / 'camera_rgb.jpg'))
    if raw is None:
        raise ValueError('agent_rgb_missing')
    overlay = raw.copy()
    choices = []
    for option in options:
        c = next((c for c in choices if c['object_id'] == option['object_id']), None)
        if c is None:
            c = dict(object_id=option['object_id'], manipulation_class='bowl',
                     grasp_method='rim_pinch', candidate_count=0, slots=[])
            choices.append(c)
        c['candidate_count'] = len({o['candidate_id'] for o in options if o['object_id'] == c['object_id']})
        if not any(s['slot_id'] == option['slot_id'] for s in c['slots']):
            c['slots'].append(dict(slot_id=option['slot_id'], planning_status='geometry_only'))
    for obj in vision.get('objects', []) + vision.get('rejected', []):
        box = obj.get('pixel_bbox')
        if box:
            x, y, x2, y2 = box
            cv2.rectangle(overlay, (x, y), (x2, y2), (0, 220, 255), 2)
            cv2.putText(overlay, f"#{obj['object_id']}", (x, max(20, y-6)), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 220, 255), 2)
    tray = vision.get('tray') or {}
    if tray.get('tray_polygon_px'):
        polygon = np.array(tray['tray_polygon_px'], dtype=np.int32)
        cv2.polylines(overlay, [polygon], True, (0, 255, 0), 2)
    # Project actual legal slot centers using the same acquisition-time transform.
    if options:
        mapping = np.asarray(vision['tray']['base_xy_to_pixel'])
        def pixel(xy):
            uv = mapping @ np.array([*xy, 1.])
            if not np.isfinite(uv).all() or abs(uv[2]) < 1e-9:
                raise ValueError('invalid_slot_projection')
            return tuple(int(round(x)) for x in uv[:2]/uv[2])
        # Labels map to exact slot IDs in JSON; numbering has no geometric meaning.
        for c in choices:
            for i, slot in enumerate(c['slots']):
                option = next(o for o in options if o['object_id'] == c['object_id'] and o['slot_id'] == slot['slot_id'])
                label = f"{c['object_id']}:S{i+1}"
                slot['label'] = label
                point = pixel(option['slot']['xy'])
                cv2.circle(overlay, point, 5, (255, 180, 0), -1)
                cv2.putText(overlay, label, (point[0]+5, point[1]-5), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 180, 0), 1)
    if not cv2.imwrite(str(directory/'agent_overlay.jpg'), overlay):
        raise OSError('agent_overlay_write_failed')
    return choices


def publish_request(directory, vision, scene, options, previous_action, home_confirmed):
    directory = Path(directory)
    choices = render_overlay(directory, vision, options)
    files = [dict(file=name, sha256=hashlib.sha256((directory/name).read_bytes()).hexdigest())
             for name in ('camera_rgb.jpg', 'agent_overlay.jpg', 'scene.json')]
    request = dict(round_protocol_version='table-round-v2',
        **{key: scene[key] for key in ('plan_id', 'task_id', 'observation_id')},
        phase='select' if choices else 'completion_review', home_confirmed=home_confirmed,
        files=files, images=files[:2], scene_sha256=files[2]['sha256'],
        rgbd_pair=vision['rgbd_pair'], previous_action=previous_action, choices=choices)
    request['scene_revision'] = request_revision(request)
    validate_request(request)
    atomic_json(directory/'agent_request.json', request)
    return request
