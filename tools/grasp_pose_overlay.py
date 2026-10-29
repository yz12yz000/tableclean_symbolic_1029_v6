"""Annotate selected target poses using the original observation's calibration."""
import math

import cv2
import numpy as np

from grasp_executor.cartesian import rotate
from grasp_executor.grasp import Pose, apply_grasp_offset


def selected_grasp_poses(batch, history, config):
    """Ignore trial candidates; only keep planning success or actual execution selection."""
    if batch is None:
        return []
    chosen = {}
    for item in history:
        status = item['status']
        if item['node'] != 'grasp_executor':
            continue
        if any(status.get(key) != batch[key] for key in ('task_id', 'observation_id', 'source_stamp_ns')):
            continue
        planned = status.get('status') == 'candidate_planned'
        executing = status.get('command_attempted') is True
        if not planned and not executing:
            continue
        oid, cid = status.get('object_id'), status.get('candidate_id')
        if type(oid) is not int or type(cid) is not int or oid < 0 or cid < 0:
            continue
        group = next((g for g in batch['objects'] if g['object_id'] == oid), None)
        candidate = next((c for c in group['candidates'] if c['candidate_id'] == cid), None) if group else None
        if candidate is None:
            raise ValueError('selected_pose_candidate_missing')
        if oid in chosen and chosen[oid]['candidate_id'] != cid:
            raise ValueError('selected_pose_changed_after_selection')
        pose = apply_grasp_offset(Pose(tuple(candidate['position']), tuple(candidate['orientation']),
                                      batch['frame_id'], batch['tool_frame']), config)
        x, y, z, w = pose.orientation
        rpy = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
               math.asin(max(-1., min(1., 2*(w*y-z*x)))),
               math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
        chosen[oid] = dict(object_id=oid, candidate_id=cid, position=list(pose.position),
            orientation=list(pose.orientation), rpy_deg=[math.degrees(v) for v in rpy],
            frame_id=pose.frame_id, tool_frame=pose.tool_frame,
            source_stamp_ns=batch['source_stamp_ns'],
            selection='execution_started' if executing else 'planned_only',
            method=candidate['geometry'].get('method'),
            outside_finger_axis=candidate['geometry'].get('outside_finger_axis'))
    return list(chosen.values())


def project_grasp(pose, camera, transform, image_header, shape, axis_length=.03):
    """Project the actual offset-adjusted target and its tool axes into original pixels."""
    if not camera or not transform or not image_header:
        raise ValueError('observation_projection_metadata_missing')
    if camera.get('model') != 'rectified_pinhole':
        raise ValueError('unsupported_projection_model')
    if (camera['image_stamp_ns'] != image_header['stamp_ns'] or
            camera['source_stamp_ns'] != pose['source_stamp_ns'] or
            transform['source_stamp_ns'] != pose['source_stamp_ns']):
        raise ValueError('projection_observation_stamp_mismatch')
    if (transform['source_frame'] != camera['frame_id'] or
            image_header['frame_id'] != camera['frame_id'] or
            transform['target_frame'] != pose['frame_id']):
        raise ValueError('projection_frame_mismatch')
    if (camera['height'], camera['width']) != tuple(shape[:2]):
        raise ValueError('projection_image_size_mismatch')
    k = np.asarray(camera['k'], dtype=float).reshape(3, 3)
    translation = np.asarray(transform['translation'], dtype=float)
    rotation = np.asarray(transform['rotation'], dtype=float)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or
            translation.shape != (3,) or not np.isfinite(translation).all() or
            rotation.shape != (4,) or not np.isfinite(rotation).all()):
        raise ValueError('invalid_projection_calibration')
    inverse = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
    origin = np.asarray(pose['position'], dtype=float)
    points = [origin] + [origin + rotate(pose['orientation'], axis)
                         for axis in np.eye(3) * axis_length]
    pixels = []
    for point in points:
        camera_point = np.asarray(rotate(inverse, point - translation))
        if not np.isfinite(camera_point).all() or camera_point[2] <= 1e-6:
            pixels.append(None)
        else:
            pixel = k @ camera_point
            pixels.append((pixel[:2] / pixel[2]).tolist())
    if pixels[0] is None:
        raise ValueError('selected_pose_behind_camera')
    if not (0 <= pixels[0][0] < shape[1] and 0 <= pixels[0][1] < shape[0]):
        raise ValueError('selected_pose_outside_image')
    return dict(origin_pixel=pixels[0], axis_pixels=pixels[1:], axis_length_m=axis_length)


def render_grasp_overlay(bgr, poses, camera, transform, image_header):
    output = bgr.copy()
    lines = ['SELECTED GRASP TARGET | magenta cross = grasp_center',
             'Tool axes: X red / Y green (closing axis) / Z blue']
    metadata = []
    for pose in poses:
        row = dict(pose)
        label = f'object {pose["object_id"]} / candidate {pose["candidate_id"]}'
        lines.append(f'{label} [{pose["selection"]}]')
        lines.append(f'Frame: {pose["frame_id"]} | Tool: {pose["tool_frame"]}')
        lines.append('XYZ (m): ' + ', '.join(f'{v:+.4f}' for v in pose['position']))
        lines.append('RPY (deg): ' + ', '.join(f'{v:+.1f}' for v in pose['rpy_deg']))
        if pose.get('outside_finger_axis'):
            lines.append('Outside finger: ' + pose['outside_finger_axis'])
        try:
            projected = project_grasp(pose, camera, transform, image_header, output.shape)
            row.update(projected)
            origin = tuple(int(round(v)) for v in projected['origin_pixel'])
            for pixel, colour, name in zip(projected['axis_pixels'],
                    ((0, 0, 255), (0, 220, 0), (255, 100, 0)), ('X', 'Y', 'Z')):
                if pixel is None or max(abs(v) for v in pixel) > 1e7:
                    continue
                end = tuple(int(round(v)) for v in pixel)
                visible, start, end = cv2.clipLine((0, 0, output.shape[1], output.shape[0]), origin, end)
                if visible:
                    cv2.arrowedLine(output, start, end, colour, 2, cv2.LINE_AA, tipLength=.25)
                    cv2.putText(output, name, (end[0]+3, end[1]-3), cv2.FONT_HERSHEY_SIMPLEX,
                                .5, colour, 1, cv2.LINE_AA)
            cv2.circle(output, origin, 7, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.drawMarker(output, origin, (255, 0, 255), cv2.MARKER_CROSS, 19, 2)
            cv2.putText(output, f'GRASP {pose["object_id"]}:{pose["candidate_id"]}',
                        (max(0, min(origin[0]+12, output.shape[1]-170)), max(18, origin[1]-13)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 2, cv2.LINE_AA)
        except ValueError as exc:
            row['projection_error'] = str(exc)
            lines.append('Image marker unavailable: ' + str(exc))
        metadata.append(row)
    # Put coordinate values in a panel, keeping the projected marker visible.
    width = min(output.shape[1], 670)
    height = min(output.shape[0], 12 + 21 * len(lines))
    right = output.shape[1] - width
    panel = output[:height, right:].copy()
    output[:height, right:] = (panel * .22).astype(np.uint8)
    for index, line in enumerate(lines):
        cv2.putText(output, line, (right+8, 20+21*index), cv2.FONT_HERSHEY_SIMPLEX,
                    .46, (255, 255, 255), 1, cv2.LINE_AA)
    return output, metadata
