"""Bounded tray-local disk packing, using fresh occupancy and one-step lookahead."""
import math
import numpy as np


def enumerate_grid_slots(tray, objects, used_slots, config, *, incoming_radius, remaining_radii):
    from .tray_utils import _disk_inside_polygon, _number
    if any(not isinstance(key, str) or not key.startswith('grid:') for key in used_slots):
        raise ValueError('invalid_tray:used_grid_slot')
    if tray['status'] != 'ok' or any(not o['position_valid'] for o in objects):
        return None
    radius = _number(incoming_radius, 'incoming_radius', positive=True) + config['position_tolerance']
    remaining = sorted((_number(r, 'remaining_radius', positive=True) + config['position_tolerance']
                        for r in remaining_radii), reverse=True)
    half = np.asarray(tray['size']) / 2
    center = np.asarray(tray['center_xy'])
    co, si = math.cos(tray['yaw']), math.sin(tray['yaw'])
    rotation = np.array([[co, -si], [si, co]])
    obstacles = [(np.asarray(o['center_xy']), o['radius']) for o in objects]

    def candidates(r):
        limits = half - r - config['edge_margin']
        if (limits < -1e-9).any():
            return np.empty((0, 2)), np.empty((0, 2))
        # A mathematically exact fit can be a few ulps below zero.
        limits = np.maximum(limits, 0.)
        step = max(config['grid_step'], math.sqrt(float(np.prod(2 * limits)) / 2500))
        axes = [np.unique(np.r_[np.linspace(-v, v, min(65, max(2, int(2*v/step)+1))), 0.]) for v in limits]
        uv = np.array([(u, v) for u in axes[0] for v in axes[1]])
        xy = uv @ rotation.T + center
        valid = np.ones(len(xy), bool)
        if 'observed_polygon_xy' in tray:
            valid &= np.array([_disk_inside_polygon(p, r + config['edge_margin'], tray['observed_polygon_xy'])
                               for p in xy])
        for p, other_radius in obstacles:
            valid &= np.linalg.norm(xy - p, axis=1) > r + other_radius + config['clearance']
        return uv[valid], xy[valid]

    uv, xy = candidates(radius)
    if not len(uv):
        return None
    # Distance from the reserved disk to the limiting boundary: both the
    # measured usable rectangle and the actual visible outline constrain it.
    # Vectorize polygon distances so edge ranking adds bounded, small cost.
    edge_gaps = np.min(half - np.abs(uv), axis=1) - radius
    polygon = tray.get('observed_polygon_xy')
    if polygon is not None:
        vertices = np.asarray(polygon, dtype=float)
        for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
            delta = b-a
            projection = np.clip((xy-a) @ delta / np.dot(delta, delta), 0., 1.)
            distances = np.linalg.norm(xy-(a+projection[:, None]*delta), axis=1)-radius
            edge_gaps = np.minimum(edge_gaps, distances)
    # Leave as many feasible positions as possible for the largest remaining
    # bowl. This is a bounded heuristic, not a promise of globally optimal packing.
    future = candidates(remaining[0])[1] if remaining else np.empty((0, 2))
    results = []
    for local, world, edge_gap in zip(uv, xy, edge_gaps):
        key = 'grid:' + ':'.join(f'{v:.6f}' for v in local)
        if key in used_slots:
            continue
        free = int(np.sum(np.linalg.norm(future - world, axis=1)
                          > radius + remaining[0] + config['clearance'])) if remaining else 0
        # Leave room for the next bowl first; then approach an allowed edge.
        # At equal edge clearance, prefer corners rather than the narrow-axis
        # centre. Quantization prevents float noise from choosing arbitrary sides.
        score = (free, -round(float(edge_gap), 6), round(float(local @ local), 8),
                 abs(float(local[0])), -float(local[0]), -float(local[1]))
        results.append((score, dict(id=key, uv=local.tolist(), xy=world.tolist(),
                                    reserved_radius=radius, remaining_candidate_count=free,
                                    boundary_clearance=float(edge_gap))))
    results.sort(key=lambda item: item[0], reverse=True)
    return [item[1] for item in results]


def choose_grid_slot(tray, objects, used_slots, config, *, incoming_radius, remaining_radii):
    slots = enumerate_grid_slots(tray, objects, used_slots, config,
                                 incoming_radius=incoming_radius,
                                 remaining_radii=remaining_radii)
    return slots[0] if slots else None
