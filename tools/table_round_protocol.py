"""ROS-free table-round-v2 boundary: Agent chooses object/slot, never geometry."""
import hashlib
import json
import re
from copy import deepcopy

FILES = ('camera_rgb.jpg', 'agent_overlay.jpg', 'scene.json')
IDENTITY = ('plan_id', 'task_id', 'observation_id', 'scene_revision')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def request_revision(request):
    return digest({k: v for k, v in request.items() if k != 'scene_revision'})


def validate_request(r):
    if not isinstance(r, dict) or r.get('round_protocol_version') != 'table-round-v2':
        raise ValueError('round_protocol_version')
    for key in IDENTITY:
        if not isinstance(r.get(key), str) or not r[key] or len(r[key]) > 256:
            raise ValueError('request_identity:' + key)
    if not re.fullmatch(re.escape(r['task_id']) + r'/round_[0-9]{4,}', r['observation_id']):
        raise ValueError('observation_id')
    if r.get('phase') not in ('select', 'completion_review') or r.get('home_confirmed') is not True:
        raise ValueError('request_phase_or_home')
    files = r.get('files', [])
    if len(files) != 3 or {f.get('file') for f in files} != set(FILES):
        raise ValueError('request_files')
    if any(set(f) != {'file', 'sha256'} or not re.fullmatch('[0-9a-f]{64}', f['sha256']) for f in files):
        raise ValueError('request_file_digest')
    if r.get('images') != [f for f in files if f['file'] != 'scene.json'] or r.get('scene_sha256') != next(f['sha256'] for f in files if f['file'] == 'scene.json'):
        raise ValueError('request_image_scene_digests')
    choices = r.get('choices')
    if not isinstance(choices, list) or len(choices) > 256:
        raise ValueError('choices')
    ids = set()
    for c in choices:
        if type(c.get('object_id')) is not int or c['object_id'] < 0 or c['object_id'] in ids:
            raise ValueError('object_id')
        ids.add(c['object_id'])
        if c.get('grasp_method') != 'rim_pinch' or type(c.get('candidate_count')) is not int or c['candidate_count'] < 1:
            raise ValueError('grasp_method_or_candidates')
        slots = c.get('slots', [])
        if not 1 <= len(slots) <= 3 or len({s['slot_id'] for s in slots}) != len(slots):
            raise ValueError('slots')
        for s in slots:
            if not isinstance(s['slot_id'], str) or not s['slot_id'] or s.get('planning_status') != 'geometry_only':
                raise ValueError('slot')
    if bool(choices) != (r['phase'] == 'select'):
        raise ValueError('phase_choices')
    if r['scene_revision'] != request_revision(r):
        raise ValueError('request_revision')
    return deepcopy(r)


def validate_decision(d, request):
    r = validate_request(request)
    required = {'schema_version', *IDENTITY, 'action'}
    if not isinstance(d, dict) or not required <= set(d) or set(d) - required - {'reason', 'selection'}:
        raise ValueError('decision_fields')
    if type(d['schema_version']) is not int or d['schema_version'] != 2:
        raise ValueError('decision_version')
    for key in IDENTITY:
        if d[key] != r[key]:
            raise ValueError('decision_identity:' + key)
    if not isinstance(d.get('reason', ''), str) or len(d.get('reason', '')) > 256:
        raise ValueError('decision_reason')
    allowed = {'select', 'reobserve', 'stop'} if r['phase'] == 'select' else {'finish', 'reobserve', 'stop'}
    if d['action'] not in allowed:
        raise ValueError('decision_action')
    if d['action'] == 'select':
        s = d.get('selection')
        if not isinstance(s, dict) or set(s) != {'object_id', 'slot_id'} or type(s['object_id']) is not int:
            raise ValueError('selection_fields')
        if not any(c['object_id'] == s['object_id'] and any(x['slot_id'] == s['slot_id'] for x in c['slots']) for c in r['choices']):
            raise ValueError('selection_not_available')
    elif 'selection' in d:
        raise ValueError('unexpected_selection')
    return deepcopy(d)


def validate_bundle(directory, request):
    validate_request(request)
    for f in request['files']:
        path = directory / f['file']
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != f['sha256']:
            raise ValueError('bundle_digest:' + f['file'])
    scene = json.loads((directory / 'scene.json').read_text())
    for key in ('plan_id', 'task_id', 'observation_id'):
        if scene.get(key) != request[key]:
            raise ValueError('bundle_identity:' + key)
    return request
