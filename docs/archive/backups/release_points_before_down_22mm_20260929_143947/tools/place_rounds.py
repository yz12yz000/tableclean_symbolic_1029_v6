"""ROS-free policies for bounded, fresh-observation pick/place rounds."""
from copy import deepcopy
from dataclasses import dataclass, replace
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
    """Use base/tool0 release XYZ with blue orientation and a 10 cm descent."""
    if len(coordinates) != 3 or any(type(value) not in (int, float) or not math.isfinite(value)
                                    for value in coordinates):
        raise ValueError('invalid_release_coordinates')
    blue = load_place_zone(path, 'blue', grasp)
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
    reference = dict(zip('xyz', (coordinates[0], coordinates[1], coordinates[2] + .1)))
    ready = dict(zip('xyz', (blue.place_ready_position['x'] - dx,
                             blue.place_ready_position['y'] - dy,
                             blue.place_ready_position['z'] + dz)))
    configured = replace(blue, place_reference_position=reference,
                         place_ready_position=ready, place_drop_distance=.1)
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

    def begin_round(self, now):
        if now >= self.deadline:
            raise TimeoutError('pipeline_timeout')
        if self.observation_count >= self.limits.max_observation_rounds:
            raise RuntimeError('max_observation_rounds_reached')
        if self.completed_count >= self.limits.max_completed_objects:
            raise RuntimeError('max_completed_objects_reached')
        self.observation_count += 1
        return self.observation_count

    def retry_observation(self, now):
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
