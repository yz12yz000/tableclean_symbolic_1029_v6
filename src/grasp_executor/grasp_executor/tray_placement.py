"""Hold the observed bowl centre over a tray target at the original release pose."""
import math
import numpy as np

from rim_locator.rim_detection import transform_points
from .grasp import Pose, validate_pose
from .planning_errors import CandidatePlanRejected


def candidate_place_ready(grasp, baseline_ready, config):
    p = config.tray_placement
    if p is None:
        return baseline_ready
    center = np.array([p['center'][k] for k in 'xyz'])
    contact = np.asarray(grasp.position)
    if abs(math.hypot(*(center - contact)[:2]) - p['rim_radius']) > p['contact_tolerance']:
        raise CandidatePlanRejected('tray_contact_not_on_observed_rim', stage='placement_geometry')
    grasp_rotation = transform_points(np.eye(3), [0., 0., 0.], grasp.orientation).T
    release_rotation = transform_points(np.eye(3), [0., 0., 0.], baseline_ready.orientation).T
    held_center = grasp_rotation.T @ (center - contact)
    xy = np.array([p['target_xy'][k] for k in 'xy']) - (release_rotation @ held_center)[:2]
    ready = Pose((float(xy[0]), float(xy[1]), baseline_ready.position[2]),
                 baseline_ready.orientation, baseline_ready.frame_id, baseline_ready.tool_frame)
    try:
        validate_pose(ready, config)
    except ValueError as exc:
        if str(exc) != 'pose_outside_workspace':
            raise
        raise CandidatePlanRejected(str(exc), stage='placement_workspace') from exc
    return ready
