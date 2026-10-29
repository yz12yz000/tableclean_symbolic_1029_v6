"""ROS-free policies for bounded, fresh-observation pick/place rounds."""
from copy import deepcopy
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path


PLACE_ZONES = ('左上', '左下', '中间', '右上', '右下', 'blue')
_ZONE_POSE_FIELDS = ('place_reference_position', 'place_reference_orientation',
                     'place_ready_position', 'place_ready_orientation')


def load_place_zone(path, zone, grasp):
    """Apply one taught ready pose to a copy of the existing grasp config."""
    if zone not in PLACE_ZONES:
        raise ValueError('invalid_target_zone')
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = json.load(stream)
    expected = {'schema_version', 'place_reference_frame', 'place_reference_child_frame',
                'place_ready_frame', 'place_ready_tool_frame', 'return_home', 'zones'}
    if (type(data) is not dict or set(data) != expected
            or type(data['schema_version']) is not int or data['schema_version'] != 2):
        raise ValueError('invalid_place_zones_schema')
    if (data['place_reference_frame'] != grasp.place_reference_frame
            or data['place_reference_child_frame'] != grasp.place_reference_child_frame
            or data['place_ready_frame'] != grasp.base_frame
            or data['place_ready_tool_frame'] != grasp.tool_frame):
        raise ValueError('place_zone_frame_mismatch')
    zones = data['zones']
    if type(zones) is not dict or set(zones) != set(PLACE_ZONES):
        raise ValueError('invalid_place_zones')
    selected = zones[zone]
    if type(selected) is not dict or set(selected) != {'enabled', 'place_drop_distance', *_ZONE_POSE_FIELDS}:
        raise ValueError('invalid_place_zone_fields')
    if type(selected['enabled']) is not bool:
        raise ValueError('invalid_place_zone_enabled')
    if not selected['enabled']:
        raise ValueError('place_zone_not_taught:' + zone)
    distance = selected['place_drop_distance']
    if type(distance) not in (int, float) or not math.isfinite(distance) or not 0 < distance <= .3:
        raise ValueError('invalid_place_zone_drop_distance')
    home = data['return_home']
    if type(home) is not dict or set(home) != {'joint_positions', 'position', 'orientation'}:
        raise ValueError('invalid_return_home_fields')
    if any(value is None for value in home.values()):
        raise ValueError('return_home_not_taught')
    # GraspConfig validates finite XYZ values, unit quaternions and workspace
    # bounds. Check both the release and return endpoints before ROS setup.
    from grasp_executor.grasp import build_place_drop_pose, build_place_ready_pose, build_return_home_pose
    configured = replace(grasp, **{field: selected[field] for field in _ZONE_POSE_FIELDS},
                         place_drop_distance=distance, return_home_joint_positions=home['joint_positions'],
                         return_home_position=home['position'], return_home_orientation=home['orientation'])
    build_place_drop_pose(build_place_ready_pose(configured), configured)
    build_return_home_pose(configured)
    return configured


def load_release_coordinates(path, coordinates, grasp):
    """Use base/tool0 release XYZ with blue orientation and descent distance."""
    return _release_coordinates(load_place_zone(path, 'blue', grasp), coordinates)


def _release_coordinates(blue, coordinates):
    """Apply destination XYZ to the task's already validated blue baseline."""
    if len(coordinates) != 3 or any(type(value) not in (int, float) or not math.isfinite(value)
                                    for value in coordinates):
        raise ValueError('invalid_release_coordinates')
    if (blue.place_reference_frame != 'base' or blue.place_reference_child_frame != 'tool0'
            or blue.base_frame != 'base_link' or blue.tool_frame != 'grasp_center'):
        raise ValueError('coordinate_mode_frame_mismatch')
    # The URDF fixes base_link -> base at yaw pi. Blue's calibrated ready pose
    # also includes the unchanged tool0 -> grasp_center offset. Shift it by
    # the input XYZ difference; resolve_place_pose audits this against live TF.
    baseline = blue.place_reference_position
    baseline_release_z = baseline['z'] - blue.place_drop_distance
    dx = coordinates[0] - baseline['x']
    dy = coordinates[1] - baseline['y']
    dz = coordinates[2] - baseline_release_z
    reference = dict(zip('xyz', (coordinates[0], coordinates[1], coordinates[2] + blue.place_drop_distance)))
    ready = dict(zip('xyz', (blue.place_ready_position['x'] - dx,
                             blue.place_ready_position['y'] - dy,
                             blue.place_ready_position['z'] + dz)))
    configured = replace(blue, place_reference_position=reference,
                         place_ready_position=ready)
    from grasp_executor.grasp import build_place_drop_pose, build_place_ready_pose, build_return_home_pose
    build_place_drop_pose(build_place_ready_pose(configured), configured)
    build_return_home_pose(configured)
    return configured


GEOMETRY_REASONS = frozenset({
    'empty_cloud', 'insufficient_points', 'insufficient_pointcloud',
    'rim_not_found:insufficient_candidates', 'rim_not_found:insufficient_local_points',
    'pose_not_found:ambiguous_components', 'pose_not_found:normal_direction_ambiguous',
    'pose_not_found:insufficient_wall_points', 'pose_not_found:degenerate_wall_neighborhood',
    'pose_not_found:wall_normal_out_of_range', 'pose_not_found:insufficient_wall_span',
    'pose_not_found:wall_fit_residual', 'pose_not_found:anchor_not_on_wall',
    'pose_not_found:unstable_wall_normal', 'pose_not_found:uncorroborated_wall_normal',
    'pose_not_found:no_supported_contact', 'pose_not_found:closing_axis_parallel_to_approach',
    'pose_not_found:circle_degenerate',
    'pose_not_found:circle_insufficient_points',
    'pose_not_found:circle_insufficient_inliers',
    'pose_not_found:circle_no_valid_radius',
    'pose_not_found:circle_low_inlier_ratio',
    'pose_not_found:circle_radius_out_of_range',
    'pose_not_found:circle_residual',
    'pose_not_found:circle_insufficient_coverage',
    'pose_not_found:circle_insufficient_sectors',
    'pose_not_found:circle_no_supported_radial_contact',
})


@dataclass(frozen=True)
class RoundLimits:
    max_reobservations: int = 2
    max_observation_rounds: int = 30
    max_completed_objects: int = 20

    def __post_init__(self):
        for name in ('max_reobservations', 'max_observation_rounds', 'max_completed_objects'):
            value = getattr(self, name)
            if type(value) is not int or value < (0 if name == 'max_reobservations' else 1):
                raise ValueError('invalid_limit:' + name)


@dataclass
class RoundBudget:
    deadline: float
    limits: RoundLimits
    observation_count: int = 0
    completed_count: int = 0
    reobservations_without_progress: int = 0
    pending_failure: bool = False
    unresolved_rejections: bool = False

    def __post_init__(self):
        if not math.isfinite(self.deadline):
            raise ValueError('invalid_deadline')

    def begin_round(self, now, *, allow_completion_check=False):
        if now >= self.deadline:
            raise TimeoutError('pipeline_timeout')
        if self.observation_count >= self.limits.max_observation_rounds:
            raise RuntimeError('max_observation_rounds_reached')
        if not allow_completion_check and self.completed_count >= self.limits.max_completed_objects:
            raise RuntimeError('max_completed_objects_reached')
        self.observation_count += 1
        return self.observation_count

    def retry_observation(self, now, *, record_failure=True):
        # Empty detections consume the same retry budget, but do not imply a
        # rejected target. Never erase a failure recorded by an earlier round.
        if record_failure:
            self.pending_failure = True
        if now >= self.deadline or self.reobservations_without_progress >= self.limits.max_reobservations:
            return False
        self.reobservations_without_progress += 1
        return True

    def complete_object(self, has_rejections=False):
        self.completed_count += 1
        self.reobservations_without_progress = 0
        prior_failure = self.pending_failure
        self.pending_failure = False
        # Without cross-frame identity matching, an unrelated success cannot
        # prove an earlier rejected object was handled. Keep this uncertainty.
        self.unresolved_rejections = self.unresolved_rejections or prior_failure or bool(has_rejections)

    def empty_result(self):
        if self.pending_failure or self.unresolved_rejections:
            return False, 'partial' if self.completed_count else 'failed', 'remaining_unknown'
        return True, 'completed' if self.completed_count else 'no_targets', ''


def recoverable_failure(statuses):
    """Only fully returned pre-motion feasibility/geometry failures may reobserve."""
    executor = statuses.get('grasp_executor', {})
    if executor.get('command_attempted') is True:
        return False
    failures = [(name, value) for name, value in statuses.items()
                if value.get('status') in ('failed', 'candidates_exhausted')]
    if not failures:
        return False
    for name, value in failures:
        reason = value.get('reason', '')
        if name == 'grasp_executor':
            if value.get('status') != 'candidates_exhausted' or value.get('command_attempted') is not False:
                return False
        elif name == 'yolo_vision':
            if reason != 'no_valid_depth_for_detected_targets':
                return False
        elif name == 'rim_locator':
            rejected = value.get('rejected', [])
            if (reason != 'pose_not_found:no_valid_object' or not rejected
                    or any(item.get('reason') not in GEOMETRY_REASONS for item in rejected)):
                return False
        else:
            return False
    return True


def select_candidate_groups(batch, offset, execute):
    """Preserve stable object distance ordering; never score/reorder candidates."""
    selected = deepcopy(batch)
    groups = selected['objects']
    # Locator already provides the historical distance/object-id ordering.
    groups.sort(key=lambda group: math.hypot(
        group['candidates'][0]['position'][0] + offset['x'],
        group['candidates'][0]['position'][1] + offset['y']))
    if execute:
        selected['objects'] = groups[:1]
        selected['rejected'] = []  # The original observation remains in the round journal.
    return selected


def context_matches(value, task_id, observation_id):
    return (isinstance(value, dict) and value.get('task_id') == task_id
            and value.get('observation_id') == observation_id)


class ClearTableSession:
    """Small, ROS-free per-task ledger shared by the live runner and offline demo.

    Object IDs are scoped to one observation. Only positions associate source
    objects between observations; tray-relative positions associate occupancy.
    A returned arm action reserves a slot, but only a fresh scene verifies it.
    """

    def __init__(self, config, policy=None, agent_mode=False):
        from yolo_vision.tray_utils import validate_tray_config
        self.config = validate_tray_config(config)
        if policy is not None:
            from table_agent_protocol import validate_task_policy
            policy = validate_task_policy(policy)
        self.policy = deepcopy(policy) if policy is not None else None
        self.agent_mode = agent_mode
        self.used_positions = set()
        self.last_placement = None
        self.motion_completed_count = 0
        self.verified_placed_count = 0
        self.remaining = []
        self.known_inside = []
        self.minimum_object_count = 0
        self.tray_identity = None
        self.tray_yaw = None
        self.last_stamp = 0
        self.empty_count = 0
        self._decision = None
        self.placement_confirmation_count = 0
        self.placement_confirmation_uv = None
        self.failed_options = []
        self._failed_fingerprints = set()
        self.replans_used = 0
        self._last_scene = None
        self._last_options = []
        self._last_scene_revision = ''
        self._selection_applied_revision = None

    @staticmethod
    def _distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _local(center, tray):
        dx, dy = center[0] - tray['center_xy'][0], center[1] - tray['center_xy'][1]
        c, s = math.cos(tray['yaw']), math.sin(tray['yaw'])
        return [c * dx + s * dy, -s * dx + c * dy]

    def _decision_result(self, kind, reason='', **details):
        if kind == 'observe_again' and reason != 'confirming_placement':
            self.placement_confirmation_count = 0
            self.placement_confirmation_uv = None
        value = dict(kind=kind, reason=reason, **details)
        self._decision = deepcopy(value) if kind == 'ready' else None
        return value

    @staticmethod
    def option_fingerprint(source, candidate, slot):
        """Identify the same physical option across modest scene re-observations."""
        def quantized(values, step=.005):
            return [round(float(value) / step) for value in values]
        geometry = candidate.get('geometry', {}) if isinstance(candidate, dict) else {}
        payload = {
            'source_xy': quantized(source.get('center_xy', ())),
            'source_radius': round(float(source.get('radius', 0.)) / .005),
            'candidate_position': quantized(candidate.get('position', ()), .001),
            'candidate_orientation': quantized(candidate.get('orientation', ()), .002),
            'candidate_geometry': geometry,
            'slot_id': slot.get('id'),
            'slot_uv': quantized(slot.get('uv', ()), .005),
        }
        encoded = json.dumps(payload, sort_keys=True, ensure_ascii=False,
                             separators=(',', ':'), allow_nan=False)
        return hashlib.sha256(encoded.encode('utf-8')).hexdigest()

    def register_plan_failure(self, decision, result):
        """Remember pre-motion failures so a fresh frame cannot repeat them forever."""
        if not isinstance(decision, dict) or decision.get('kind') != 'ready':
            return
        if not isinstance(result, dict) or result.get('command_attempted') is True:
            return
        failures = result.get('candidate_failures') or []
        selected = decision.get('selected', {}).get('objects', [])
        if len(selected) != 1:
            return
        candidates = {c.get('candidate_id'): c for c in selected[0].get('candidates', [])}
        source = decision.get('source', {})
        slot = decision.get('slot', {})
        for failure in failures:
            candidate = candidates.get(failure.get('candidate_id'))
            if candidate is None:
                continue
            fingerprint = self.option_fingerprint(source, candidate, slot)
            if fingerprint in self._failed_fingerprints:
                continue
            self._failed_fingerprints.add(fingerprint)
            self.replans_used += 1
            self.failed_options.append({
                'fingerprint': fingerprint,
                'scene_revision': self._last_scene_revision,
                'object_id': failure.get('object_id'),
                'candidate_id': failure.get('candidate_id'),
                'slot_id': slot.get('id'),
                'stage': failure.get('stage', ''),
                'reason': failure.get('reason', ''),
            })

    def option_blocked(self, source, candidate, slot):
        return self.option_fingerprint(source, candidate, slot) in self._failed_fingerprints

    def list_options(self):
        """Return current-round references suitable for an external Agent."""
        return deepcopy(self._last_options)

    def current_scene_revision(self):
        return self._last_scene_revision

    def apply_selection(self, decision):
        """Bind one Agent reference to the last fresh scene.

        This method never accepts coordinates. It returns a normal ``ready``
        decision whose non-selected contacts are disabled without renumbering
        candidate IDs, so the existing strict candidate protocol remains valid.
        """
        from table_agent_protocol import validate_round_decision
        if self._last_scene is None:
            raise ValueError('table_agent:no_current_scene')
        if self._selection_applied_revision == self._last_scene.get('scene_revision'):
            raise ValueError('table_agent:decision_already_applied')
        decision = validate_round_decision(
            decision, task_id=self._last_scene.get('task_id'),
            plan_id=self._last_scene.get('plan_id'),
            observation_id=self._last_scene.get('observation_id'),
            scene_revision=self._last_scene.get('scene_revision'))
        selection = decision['selection']
        matches = [option for option in self._last_options
                   if option['object_id'] == selection['object_id']
                   and option['candidate_id'] == selection['candidate_id']
                   and option['slot_id'] == selection['slot_id']]
        if len(matches) != 1:
            raise ValueError('table_agent:selection_not_available')
        option = matches[0]
        batch = deepcopy(self._last_scene['batch'])
        groups = [group for group in batch['objects'] if group['object_id'] == option['object_id']]
        if len(groups) != 1:
            raise ValueError('table_agent:object_not_available')
        group = groups[0]
        group['candidates'] = [dict(candidate,
                                    disabled=(candidate['candidate_id'] != option['candidate_id']))
                               for candidate in group['candidates']]
        batch['objects'], batch['rejected'] = [group], []
        source = deepcopy(self._last_scene['sources'][option['object_id']])
        slot = deepcopy(option['slot'])
        result = self._decision_result(
            'ready', selected=batch, slot=slot, source=source,
            inside_uv=deepcopy(self._last_scene['inside_uv']),
            tray=deepcopy(self._last_scene['tray']), verification=None,
            agent_selection=selection,
            scene_revision=self._last_scene.get('scene_revision'))
        self._selection_applied_revision = self._last_scene.get('scene_revision')
        return result

    def apply_object_slot(self, selection):
        """Internal binding after v2 validation; retain every available local contact."""
        if self._last_scene is None or self._selection_applied_revision == self._last_scene_revision:
            raise ValueError('table_agent:no_scene_or_consumed')
        matches = [o for o in self._last_options if o['object_id'] == selection['object_id']
                   and o['slot_id'] == selection['slot_id']]
        if not matches:
            raise ValueError('table_agent:selection_not_available')
        scene = self._last_scene
        batch = deepcopy(scene['batch'])
        group = next(g for g in batch['objects'] if g['object_id'] == selection['object_id'])
        available = {o['candidate_id'] for o in matches}
        group['candidates'] = [dict(c, disabled=c.get('disabled', False) or c['candidate_id'] not in available)
                               for c in group['candidates']]
        batch['objects'], batch['rejected'] = [group], []
        source = deepcopy(scene['sources'][selection['object_id']])
        source['source_stamp_ns'] = batch['source_stamp_ns']
        result = self._decision_result('ready', selected=batch, slot=deepcopy(matches[0]['slot']),
            source=source, inside_uv=deepcopy(scene['inside_uv']), tray=deepcopy(scene['tray']),
            verification=None, agent_selection=deepcopy(selection), scene_revision=self._last_scene_revision)
        self._selection_applied_revision = self._last_scene_revision
        return result

    def observe(self, vision, batch=None):
        from yolo_vision.tray_utils import (validate_tray_observation, validate_objects,
                                            classify_object, enumerate_safe_slots, tray_polygon)
        from table_agent_protocol import scene_revision
        stamp = vision.get('source_stamp_ns')
        if type(stamp) is not int or stamp <= self.last_stamp:
            raise ValueError('clear_table_observation_not_fresh')
        self.last_stamp = stamp
        # Invalidate the previous Agent binding as soon as a new frame arrives.
        # Even a not-found or incomplete frame must never reuse its old scene
        # revision or make an old selection executable.
        self._last_scene = None
        self._last_options = []
        self._selection_applied_revision = None
        config_hash = scene_revision(self.config)
        self._last_scene_revision = scene_revision({
            'config_hash': config_hash,
            'tray': vision.get('tray'),
            'objects': vision.get('objects', []),
            'rejected': vision.get('rejected', []),
            'obstacles': vision.get('obstacles', []),
            'batch': batch,
        })
        tray = validate_tray_observation(vision.get('tray'), self.config)
        if tray['status'] != 'ok':
            self.empty_count = 0
            return self._decision_result('observe_again', tray.get('reason') or 'tray_not_found')
        # A rectangular mask provides an axis, not a directed heading. Keep
        # its equivalent yaw nearest the prior frame so crossing +/-90 degrees
        # cannot exchange occupied slot IDs. A real 180-degree turn cannot be
        # inferred from an unmarked rectangle: only small inter-round turns
        # of the same tray are supported.
        if self.tray_yaw is not None:
            difference = tray['yaw'] - self.tray_yaw
            delta = .5 * math.atan2(math.sin(2 * difference), math.cos(2 * difference))
            if abs(delta) > math.pi / 4:
                self.empty_count = 0
                return self._decision_result('observe_again', 'tray_rotation_too_large')
            tray['yaw'] = self.tray_yaw + delta
            tray['polygon_xy'] = tray_polygon(tray['center_xy'], tray['yaw'], tray['size'])
        objects = validate_objects(vision.get('objects', []) + vision.get('rejected', []), self.config)
        scene_obstacles = validate_objects(vision.get('obstacles', []), self.config)
        if any(not obj['position_valid'] for obj in scene_obstacles):
            return self._decision_result('observe_again', 'obstacle_position_uncertain')
        if (type(vision.get('detected_count')) is not int or type(vision.get('object_count')) is not int
                or vision['detected_count'] != len(objects) or vision['object_count'] != len(vision.get('objects', []))):
            raise ValueError('clear_table_observation_count_mismatch')
        accepted = {obj['object_id'] for obj in vision.get('objects', [])}
        if batch is not None:
            from rim_locator.candidate_protocol import validate_candidate_batch
            batch = validate_candidate_batch(batch)
            if (batch['source_stamp_ns'] != stamp
                    or not context_matches(batch, vision.get('task_id'), vision.get('observation_id'))):
                raise ValueError('clear_table_batch_context_mismatch')
            located = {obj['object_id'] for obj in batch['objects'] + batch['rejected']}
            if located != accepted:
                raise ValueError('perception_object_mismatch')
        elif accepted:
            raise ValueError('clear_table_missing_candidate_batch')
        if self.tray_identity is not None and tray['identity'] != self.tray_identity:
            return self._decision_result('stop', 'tray_identity_changed')
        self.tray_identity = tray['identity']
        self.tray_yaw = tray['yaw']
        # Observed tray occupants need center membership, not a fresh admission
        # check for their full uncertainty disk. The same list feeds placement
        # confirmation and occupancy memory; their avoidance radii stay intact.
        regions = {obj['object_id']: classify_object(obj, tray, self.config, observed_occupancy=True)
                   for obj in objects}
        sources = [obj for obj in objects if regions[obj['object_id']] == 'outside']
        inside = [obj for obj in objects if regions[obj['object_id']] == 'inside']
        uncertain = [obj['object_id'] for obj in objects if regions[obj['object_id']] == 'unknown']
        tolerance = self.config['observation_match_tolerance']
        verification = None
        relevant_count = sum(region != 'out_of_source' for region in regions.values())
        self.minimum_object_count = max(self.minimum_object_count, relevant_count)
        if batch:
            for rejected in batch['rejected']:
                if rejected['reason'] not in GEOMETRY_REASONS | {'output_z_outside_workspace'}:
                    return self._decision_result('stop', 'localization_rejected:' + rejected['reason'])
        unmatched_inside = set(range(len(self.known_inside)))
        # Retain observed non-target occupancy inside the tray as well. A
        # momentary missed cup must not turn its last known position into space.
        occupied_obstacles = [obj for obj in scene_obstacles
                              if classify_object(obj, tray, self.config, observed_occupancy=True) == 'inside']
        for obj in inside + occupied_obstacles:
            uv = self._local(obj['center_xy'], tray)
            matches = [index for index in unmatched_inside
                       if self._distance(self.known_inside[index]['uv'], uv) <= tolerance]
            if matches:
                index = min(matches, key=lambda i: self._distance(self.known_inside[i]['uv'], uv))
                unmatched_inside.remove(index)
                self.known_inside[index]['uv'] = uv
                self.known_inside[index]['radius'] = max(self.known_inside[index]['radius'], obj['radius'])
            else:
                self.known_inside.append(dict(uv=uv, radius=obj['radius']))

        # Remember every observed source, including a bowl without grasp geometry.
        # A later empty frame cannot erase an earlier unhandled bowl.
        unmatched = set(range(len(self.remaining)))
        for obj in sources:
            matches = [i for i in unmatched if self._distance(self.remaining[i], obj['center_xy']) <= tolerance]
            if matches:
                index = min(matches, key=lambda i: self._distance(self.remaining[i], obj['center_xy']))
                unmatched.remove(index)
                self.remaining[index] = list(obj['center_xy'])
            else:
                self.remaining.append(list(obj['center_xy']))

        if uncertain:
            self.empty_count = 0
            return self._decision_result('observe_again', 'object_position_uncertain', unknown_ids=uncertain)
        if relevant_count < self.minimum_object_count:
            self.empty_count = 0
            return self._decision_result('observe_again', 'observation_objects_missing')
        if self.last_placement is not None:
            previous = self.last_placement
            now_uv = [self._local(obj['center_xy'], tray) for obj in inside + occupied_obstacles]
            # Match old occupancy first, then require a newly observed occupant
            # at the chosen slot. An existing nearby bowl is not placement proof.
            unused = set(range(len(now_uv)))
            old_present = True
            for old in previous['inside_uv']:
                matches = [i for i in unused if self._distance(now_uv[i], old) <= tolerance]
                if not matches:
                    old_present = False
                    break
                unused.remove(min(matches, key=lambda i: self._distance(now_uv[i], old)))
            placement_matches = [i for i in unused if i < len(inside)
                                if self._distance(now_uv[i], previous['slot']['uv'])
                                <= self.config['placement_match_tolerance']
                                and abs(inside[i]['radius'] - previous['source']['radius']) <= tolerance]
            new_at_slot = len(placement_matches) == 1
            source_present = any(self._distance(obj['center_xy'], previous['source']['center_xy']) <= tolerance
                                 for obj in sources)
            verification = dict(source_absent=not source_present, new_at_slot=new_at_slot,
                                previous_occupancy_preserved=old_present)
            if source_present or not new_at_slot or not old_present:
                self.empty_count = 0
                return self._decision_result('observe_again', 'placement_unverified', verification=verification)
            actual_uv = now_uv[placement_matches[0]]
            if (self.placement_confirmation_uv is not None
                    and self._distance(actual_uv, self.placement_confirmation_uv) <= tolerance):
                self.placement_confirmation_count += 1
            else:
                self.placement_confirmation_count = 1
            self.placement_confirmation_uv = actual_uv
            verification.update(consecutive_confirmations=self.placement_confirmation_count,
                                observed_uv=actual_uv,
                                placement_error=self._distance(actual_uv, previous['slot']['uv']))
            if self.placement_confirmation_count < self.config['placement_confirmations']:
                return self._decision_result('observe_again', 'confirming_placement', verification=verification)
            matches = [i for i, center in enumerate(self.remaining)
                       if self._distance(center, previous['source']['center_xy']) <= tolerance]
            if matches:
                del self.remaining[min(matches, key=lambda i: self._distance(self.remaining[i], previous['source']['center_xy']))]
            self.verified_placed_count += 1
            self.last_placement = None
            self.placement_confirmation_count = 0
            self.placement_confirmation_uv = None

        if not sources:
            if self.remaining:
                self.empty_count = 0
                return self._decision_result('observe_again', 'remaining_unknown', verification=verification)
            self.empty_count += 1
            if self.empty_count < self.config['empty_confirmations']:
                return self._decision_result('observe_again', 'confirming_source_empty', verification=verification)
            return self._decision_result('complete', 'completed' if self.verified_placed_count else 'no_targets',
                                         verification=verification)
        self.empty_count = 0
        source_ids = {obj['object_id'] for obj in sources}
        available = [group for group in batch['objects'] if group['object_id'] in source_ids] if batch else []
        if not available:
            return self._decision_result('observe_again', 'source_candidates_exhausted', verification=verification)
        # Keep the existing source ordering, then size its destination using
        # the observed footprint. Grid mode adds future placement uncertainty.
        ordered = list(available)
        largest_first = (self.policy is None or
                         self.policy.get('priority', {}).get('order') == 'largest_first')
        if largest_first and (self.config.get('known_bowl_radii') or self.config.get('object_geometry') == 'visual_rim'):
            # On a tight measured tray, a small bowl must not take the only
            # destination that can hold a larger one. Place the largest first;
            # preserve original distance order for ties and all contact poses.
            radii = {obj['object_id']: obj['radius'] for obj in sources}
            ordered.sort(key=lambda item: radii[item['object_id']], reverse=True)
        # Keep initially occupied locations too, even if an unrelated newly
        # detected object happens to mask a temporary count drop elsewhere.
        obstacles = deepcopy(objects)
        next_id = max((obj['object_id'] for obj in objects), default=-1) + 1
        for obj in scene_obstacles:
            obstacles.append(dict(obj, object_id=next_id))
            next_id += 1
        c, s = math.cos(tray['yaw']), math.sin(tray['yaw'])
        for index, record in enumerate(self.known_inside):
            u, v = record['uv']
            obstacles.append(dict(object_id=next_id + index, position_valid=True, frame_id=self.config['frame_id'],
                                  center_xy=[tray['center_xy'][0] + c*u - s*v,
                                             tray['center_xy'][1] + s*u + c*v], radius=record['radius']))
        # Enumerate safe slots and skip only combinations that have already
        # failed before motion. A rejected contact never reserves a slot. If
        # one object has no remaining option, try the next source object.
        selected_group, selected_source, slot, usable = None, None, None, None
        options = []
        saw_slot = False
        max_replans = (self.policy or {}).get('limits', {}).get('max_replans')
        if max_replans is not None and self.replans_used > max_replans:
            return self._decision_result('stop', 'replan_budget_exhausted', verification=verification)
        for group_index, group in enumerate(ordered):
            source = next(obj for obj in sources if obj['object_id'] == group['object_id'])
            source = deepcopy(source)
            source['source_stamp_ns'] = stamp
            slots = enumerate_safe_slots(
                tray, obstacles, self.used_positions, self.config,
                incoming_radius=source['radius'],
                remaining_radii=[o['radius'] for o in sources if o['object_id'] != source['object_id']])
            if not slots and group_index == 0 and not self.agent_mode:
                # A physically oversized first target is a capacity failure;
                # do not silently reorder it behind a smaller bowl. Reordering
                # is reserved for a concrete pre-motion option failure.
                return self._decision_result('stop', 'tray_full', verification=verification)
            if self.agent_mode:
                slots = slots[:3]
            saw_slot = saw_slot or bool(slots)
            for option_slot in slots:
                for option_candidate in group['candidates']:
                    if not option_candidate.get('disabled', False) and not self.option_blocked(source, option_candidate, option_slot):
                        options.append(dict(object_id=group['object_id'],
                                            candidate_id=option_candidate['candidate_id'],
                                            slot_id=option_slot['id'], slot=deepcopy(option_slot)))
            for candidate_slot in (slots if selected_group is None else []):
                candidates = [candidate for candidate in group['candidates']
                              if not candidate.get('disabled', False) and not self.option_blocked(source, candidate, candidate_slot)]
                if candidates:
                    selected_group, selected_source, slot, usable = group, source, candidate_slot, candidates
                    break
            if selected_group is not None and not self.agent_mode:
                break
        if selected_group is None:
            if max_replans is not None and self.replans_used > max_replans:
                return self._decision_result('stop', 'replan_budget_exhausted', verification=verification)
            return self._decision_result('stop' if not saw_slot else 'observe_again',
                                         'tray_full' if not saw_slot else 'all_options_exhausted',
                                         verification=verification)
        # Locator/runner already supplies distance ordering; don't reorder contacts.
        selected = deepcopy(batch)
        selected['objects'] = [deepcopy(selected_group)]
        usable_ids = {candidate['candidate_id'] for candidate in usable}
        selected['objects'][0]['candidates'] = [
            dict(candidate, disabled=(candidate['candidate_id'] not in usable_ids))
            if candidate['candidate_id'] not in usable_ids else candidate
            for candidate in selected['objects'][0]['candidates']]
        selected['rejected'] = []
        self._last_scene_revision = scene_revision({
            'config_hash': config_hash, 'tray': tray, 'objects': objects,
            'obstacles': scene_obstacles, 'batch': batch})
        self._last_scene = dict(
            task_id=vision.get('task_id', ''), plan_id=(self.policy or {}).get('plan_id', ''),
            observation_id=vision.get('observation_id', ''), scene_revision=self._last_scene_revision,
            config_hash=config_hash,
            batch=deepcopy(batch), sources={obj['object_id']: deepcopy(obj) for obj in sources},
            inside_uv=[list(record['uv']) for record in self.known_inside],
            tray=deepcopy(tray))
        self._last_options = options
        self._selection_applied_revision = None
        return self._decision_result('ready', selected=selected, slot=slot, source=selected_source,
                                     inside_uv=[list(record['uv']) for record in self.known_inside],
                                     tray=tray, verification=verification,
                                     scene_revision=self._last_scene_revision)

    def record_motion(self, decision):
        """Record a successfully returned single-object action; never call for preview."""
        if decision.get('kind') != 'ready' or self._decision != decision or self.last_placement is not None:
            raise ValueError('invalid_clear_table_motion_commit')
        self.used_positions.add(decision['slot']['id'])
        self.last_placement = deepcopy({key: decision[key] for key in ('source', 'slot', 'inside_uv')})
        self.motion_completed_count += 1
        self._decision = None

    def summary(self):
        return dict(motion_completed_count=self.motion_completed_count,
                    verified_placed_count=self.verified_placed_count,
                    used_positions=sorted(self.used_positions), last_placement=deepcopy(self.last_placement),
                    remaining_sources=deepcopy(self.remaining), empty_confirmations=self.empty_count,
                    minimum_object_count=self.minimum_object_count,
                    failed_options=deepcopy(self.failed_options),
                    replans_used=self.replans_used,
                    policy=deepcopy(self.policy))


def visual_tray_grasp_config(decision, blue, tray_config):
    """Bind visual geometry to one observation; executor compensates each contact."""
    if (blue.base_frame != 'base_link' or blue.tool_frame != 'grasp_center'
            or blue.place_reference_frame != 'base' or blue.place_reference_child_frame != 'tool0'):
        raise ValueError('coordinate_mode_frame_mismatch')
    source, selected = decision['source'], decision['selected']
    if (source.get('position_reason') != 'visual_rim_circle' or not source.get('position_valid')
            or source.get('frame_id') != 'base' or len(selected['objects']) != 1
            or selected['objects'][0]['object_id'] != source['object_id']):
        raise ValueError('visual_tray_requires_observed_geometry')
    z = tray_config['release_z']
    if z is not None:
        blue = _release_coordinates(blue, (blue.place_reference_position['x'],
                                           blue.place_reference_position['y'], z))
    target = [a + b for a, b in zip(decision['slot']['xy'], tray_config['release_xy_offset'])]
    return replace(blue, tray_placement=dict(object_id=source['object_id'],
        source_stamp_ns=selected['source_stamp_ns'],
        center=dict(x=-source['center_xy'][0], y=-source['center_xy'][1], z=source['center_z']),
        target_xy=dict(x=-target[0], y=-target[1]),
        rim_radius=source['observed_rim_radius'], contact_tolerance=.015))


def tray_release_config(slot, tray_config, blue, *, source=None, candidate=None):
    """Change destination XY (and optional existing Z override), not orientation.

    Measured-bowl mode keeps the source geometry in ``tray_placement`` so the
    executor can preserve the held rim-to-centre vector for each candidate.
    The legacy ``candidate=`` argument remains supported for callers that need
    the old precomputed release pose.
    """
    z = tray_config['release_z']
    if z is None:
        z = blue.place_reference_position['z'] - blue.place_drop_distance
    xy = [a + b for a, b in zip(slot['xy'], tray_config['release_xy_offset'])]
    if tray_config.get('compensate_bowl_xy'):
        import numpy as np
        from rim_locator.rim_detection import transform_points
        if (not source or source.get('frame_id') != 'base'
                or source.get('position_reason') != 'measured_rim_circle'
                or not source.get('position_valid')
                or source.get('measured_radius') not in tray_config['known_bowl_radii']):
            raise ValueError('bowl_xy_compensation_requires_measured_source_and_contact')
        # Keep the placement geometry attached to the observation instead of
        # baking the first contact into the release pose.  The executor can
        # then recompute the held centre for every contact it validates.
        if candidate is None:
            stamp = source.get('source_stamp_ns')
            if type(stamp) is not int or stamp <= 0:
                raise ValueError('bowl_xy_compensation_requires_source_stamp')
            target = [a + b for a, b in zip(slot['xy'], tray_config['release_xy_offset'])]
            return replace(blue, tray_placement=dict(
                object_id=source['object_id'], source_stamp_ns=stamp,
                center=dict(x=-source['center_xy'][0], y=-source['center_xy'][1], z=source['center_z']),
                target_xy=dict(x=-target[0], y=-target[1]),
                rim_radius=source['measured_radius'], contact_tolerance=.015))
        center = np.array([-source['center_xy'][0], -source['center_xy'][1], source['center_z']])
        contact = np.array(candidate['position']) + [blue.grasp_offset[k] for k in 'xyz']
        if not np.isfinite(center).all() or not np.isfinite(contact).all():
            raise ValueError('bowl_xy_compensation_invalid_center')
        grasp_rotation = transform_points(np.eye(3), [0., 0., 0.], candidate['orientation']).T
        release_rotation = transform_points(np.eye(3), [0., 0., 0.],
                                             [blue.place_ready_orientation[k] for k in 'xyzw']).T
        held_center = grasp_rotation.T @ (center - contact)
        if abs(np.linalg.norm((center - contact)[:2]) - source['measured_radius']) > .015:
            raise ValueError('bowl_xy_compensation_contact_not_on_measured_rim')
        # base_link -> base is yaw pi. The two taught poses also include the
        # tool0 -> grasp_center offset at the unchanged release orientation.
        correction = ((release_rotation @ held_center)[:2]
                      + [blue.place_ready_position[k] + blue.place_reference_position[k] for k in 'xy'])
        xy = (np.asarray(xy) + correction).tolist()
    return _release_coordinates(blue, (*xy, z))
