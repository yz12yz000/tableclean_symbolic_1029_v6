"""Small, ROS-free helpers for the optional clear-table mode.

All lengths are metres and all angles radians. Slots ``uv`` are offsets from
the tray centre, not fractions. A YOLO instance mask locates one rectangular
tray; registered RGB-D points and a rigid camera-to-base transform come from
the same observation. tray_outer_size describes the detected outer contour,
while tray_size is its concentric usable interior. The long axis is canonical
modulo pi; session code must preserve orientation continuity between rounds.

These helpers only choose destinations. They never change grasp candidates,
place poses, motion planning, or collision scenes.
"""

from copy import deepcopy
import json
import math
from numbers import Real
from pathlib import Path

import numpy as np


def _number(value, name, *, positive=False, nonnegative=False):
    if (isinstance(value, bool) or not isinstance(value, Real)
            or not math.isfinite(value) or (positive and value <= 0)
            or (nonnegative and value < 0)):
        raise ValueError("invalid_tray:" + name)
    return float(value)


def _vector(value, length, name):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length:
        raise ValueError("invalid_tray:" + name)
    return [_number(v, name) for v in value]


def _text(value, name):
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError("invalid_tray:" + name)
    return value


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segment_distance(point, a, b):
    p, a, b = np.asarray(point), np.asarray(a), np.asarray(b)
    delta = b - a
    t = np.clip(np.dot(p - a, delta) / np.dot(delta, delta), 0, 1)
    return float(np.linalg.norm(p - (a + t * delta)))


def _polygon(value, name):
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError("invalid_tray:" + name)
    vertices = [_vector(p, 2, name) for p in value]
    if len(set(map(tuple, vertices))) != len(vertices):
        raise ValueError("invalid_tray:" + name + ":duplicate_vertex")
    n = len(vertices)
    area2 = sum(vertices[i][0] * vertices[(i + 1) % n][1]
                - vertices[(i + 1) % n][0] * vertices[i][1] for i in range(n))
    if abs(area2) < 1e-10:
        raise ValueError("invalid_tray:" + name + ":zero_area")
    for i in range(n):
        a, b = vertices[i], vertices[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            c, d = vertices[j], vertices[(j + 1) % n]
            if (_cross(a, b, c) * _cross(a, b, d) <= 0
                    and _cross(c, d, a) * _cross(c, d, b) <= 0
                    and max(min(a[0], b[0]), min(c[0], d[0])) <= min(max(a[0], b[0]), max(c[0], d[0]))
                    and max(min(a[1], b[1]), min(c[1], d[1])) <= min(max(a[1], b[1]), max(c[1], d[1]))):
                raise ValueError("invalid_tray:" + name + ":self_intersection")
    return vertices


def validate_tray_config(config, *, require_calibrated=False):
    """Validate configuration; live preview also requires measured calibration."""
    if not isinstance(config, dict):
        raise ValueError("invalid_tray:config")
    c = deepcopy(config)
    if type(c.get("schema_version")) is not int or c["schema_version"] != 1:
        raise ValueError("invalid_tray:schema_version")
    if type(c.get("calibrated")) is not bool:
        raise ValueError("invalid_tray:calibrated")
    # Calibration describes measured geometry; motion approval is a separate
    # field so a newly edited field file cannot silently authorize execution.
    c.setdefault("motion_approved", False)
    if type(c["motion_approved"]) is not bool:
        raise ValueError("invalid_tray:motion_approved")
    # The existing XYZ release interface uses base/tool0, not base_link.
    if c.get("frame_id") != "base":
        raise ValueError("invalid_tray:frame_id:requires_base")
    if c.get("detector") != "yolo":
        raise ValueError("invalid_tray:detector:requires_yolo")
    yolo = c.get("yolo")
    if not isinstance(yolo, dict):
        raise ValueError("invalid_tray:yolo")
    for key in ("model_path", "target_class", "device"):
        yolo[key] = _text(yolo.get(key), "yolo." + key)
    yolo["confidence"] = _number(yolo.get("confidence"), "yolo.confidence", positive=True)
    if yolo["confidence"] > 1:
        raise ValueError("invalid_tray:yolo.confidence")
    if type(yolo.get("imgsz")) is not int or yolo["imgsz"] < 32 or yolo["imgsz"] % 32:
        raise ValueError("invalid_tray:yolo.imgsz")
    c["tray_size"] = _vector(c.get("tray_size"), 2, "tray_size")
    c["tray_outer_size"] = _vector(c.get("tray_outer_size"), 2, "tray_outer_size")
    if min(c["tray_size"]) <= 0 or min(c["tray_outer_size"]) <= 0:
        raise ValueError("invalid_tray:tray_size")
    if c["tray_outer_size"][0] / c["tray_outer_size"][1] < 1.12:
        raise ValueError("invalid_tray:tray_outer_size:requires_distinct_long_axis")
    if any(inner > outer for inner, outer in zip(c["tray_size"], c["tray_outer_size"])):
        raise ValueError("invalid_tray:tray_size:outside_outer_contour")
    for key in ("bowl_radius", "edge_margin", "clearance", "position_tolerance"):
        c[key] = _number(c.get(key), key, positive=(key == "bowl_radius"), nonnegative=True)
    # Keep the original 6 mm default. A measured camera noise allowance may
    # consume existing position tolerance, but must never exceed 10 mm.
    c["depth_xy_tolerance"] = _number(c.get("depth_xy_tolerance", .006),
                                       "depth_xy_tolerance", positive=True)
    if c["depth_xy_tolerance"] > min(.010, max(.006, c["position_tolerance"])):
        raise ValueError("invalid_tray:depth_xy_tolerance:exceeds_position_allowance")
    c.setdefault("refine_boundary", False)
    if type(c["refine_boundary"]) is not bool:
        raise ValueError("invalid_tray:refine_boundary")
    c.setdefault("object_geometry", "legacy")
    c.setdefault("placement_mode", "slots")
    if c["object_geometry"] not in ("legacy", "visual_rim"):
        raise ValueError("invalid_tray:object_geometry")
    for key, default in (("visual_rim_min_radius", .04), ("visual_rim_max_radius", .15)):
        c[key] = _number(c.get(key, default), key, positive=True)
    if not .02 <= c['visual_rim_min_radius'] < c['visual_rim_max_radius'] <= .15:
        raise ValueError('invalid_tray:visual_rim_radius_range')
    if c["placement_mode"] not in ("slots", "grid"):
        raise ValueError("invalid_tray:placement_mode")
    for key, default, low, high in (("grid_step", .01, .005, .05),
                                   ("observation_match_tolerance", .025, .001, .08),
                                   ("rim_outer_margin", .004, 0., .015),
                                   ("placement_match_tolerance", c["position_tolerance"], .001, .08)):
        c[key] = _number(c.get(key, default), key, nonnegative=True)
        if not low <= c[key] <= high:
            raise ValueError("invalid_tray:" + key)
    c.setdefault("placement_confirmations", 1)
    if type(c["placement_confirmations"]) is not int or not 1 <= c["placement_confirmations"] <= 5:
        raise ValueError("invalid_tray:placement_confirmations")
    radii = c.setdefault("known_bowl_radii", [])
    if not isinstance(radii, list):
        raise ValueError("invalid_tray:known_bowl_radii")
    c["known_bowl_radii"] = [_number(r, "known_bowl_radii", positive=True) for r in radii]
    if len(set(radii)) != len(radii) or any(r < c["bowl_radius"] for r in radii):
        raise ValueError("invalid_tray:known_bowl_radii")
    c.setdefault("compensate_bowl_xy", False)
    if type(c["compensate_bowl_xy"]) is not bool:
        raise ValueError("invalid_tray:compensate_bowl_xy")
    if c["object_geometry"] == "visual_rim" and radii:
        raise ValueError("invalid_tray:visual_geometry_does_not_use_known_radii")
    if c["compensate_bowl_xy"] and not radii and c["object_geometry"] != "visual_rim":
        raise ValueError("invalid_tray:compensation_requires_measured_radii")
    c["release_xy_offset"] = _vector(c.get("release_xy_offset"), 2, "release_xy_offset")
    if c.get("release_z") is not None:
        c["release_z"] = _number(c["release_z"], "release_z")
    if type(c.get("empty_confirmations")) is not int or c["empty_confirmations"] < 2:
        raise ValueError("invalid_tray:empty_confirmations")
    slots = c.get("slots")
    if not isinstance(slots, list) or (not slots and c["placement_mode"] == "slots"):
        raise ValueError("invalid_tray:slots")
    ids, locations = set(), set()
    for slot in slots:
        if not isinstance(slot, dict):
            raise ValueError("invalid_tray:slot")
        key = _text(slot.get("id"), "slot.id")
        uv = _vector(slot.get("uv"), 2, "slot.uv")
        if key in ids or tuple(uv) in locations:
            raise ValueError("invalid_tray:duplicate_slot")
        ids.add(key)
        locations.add(tuple(uv))
        slot["uv"] = uv
        margin = c["bowl_radius"] + c["position_tolerance"] + c["edge_margin"]
        if any(abs(v) + margin > size / 2 + 1e-9 for v, size in zip(uv, c["tray_size"])):
            raise ValueError("invalid_tray:slot_too_close_to_edge:" + key)
    if c.get("source_polygon") is not None:
        c["source_polygon"] = _polygon(c["source_polygon"], "source_polygon")
    if require_calibrated:
        if not c["calibrated"]:
            raise ValueError("tray_config_not_calibrated")
        if c.get("source_polygon") is None:
            raise ValueError("invalid_tray:source_polygon:required_for_live")
    return c


def load_tray_config(path, *, require_calibrated=False):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return validate_tray_config(json.load(stream), require_calibrated=require_calibrated)


def rotate_xy(xy, yaw):
    x, y = _vector(xy, 2, "xy")
    yaw = _number(yaw, "yaw")
    co, si = math.cos(yaw), math.sin(yaw)
    return [co * x - si * y, si * x + co * y]


def tray_polygon(center_xy, yaw, size):
    center = np.asarray(_vector(center_xy, 2, "center_xy"))
    width, height = _vector(size, 2, "size")
    return [(center + rotate_xy(p, yaw)).tolist() for p in
            ((-width / 2, -height / 2), (width / 2, -height / 2),
             (width / 2, height / 2), (-width / 2, height / 2))]


def validate_tray_observation(tray, config):
    """Check scene fields without rejecting unrelated diagnostic metadata."""
    if not isinstance(tray, dict) or tray.get("status") not in ("ok", "not_found", "invalid"):
        raise ValueError("invalid_tray:observation.status")
    t = deepcopy(tray)
    if t.get("frame_id") != config["frame_id"]:
        raise ValueError("invalid_tray:observation.frame_id")
    if t["status"] != "ok":
        _text(t.get("reason"), "observation.reason")
        return t
    t["center_xy"] = _vector(t.get("center_xy"), 2, "observation.center_xy")
    t["yaw"] = _number(t.get("yaw"), "observation.yaw")
    t["size"] = _vector(t.get("size"), 2, "observation.size")
    if not np.allclose(t["size"], config["tray_size"], atol=1e-8, rtol=0):
        raise ValueError("invalid_tray:observation.size_mismatch")
    expected_identity = "yolo:" + config["yolo"]["target_class"]
    if t.get("identity") != expected_identity:
        raise ValueError("invalid_tray:observation.identity")
    polygon = tray_polygon(t["center_xy"], t["yaw"], t["size"])
    if "polygon_xy" in t:
        supplied = _polygon(t["polygon_xy"], "observation.polygon_xy")
        if len(supplied) != 4 or not np.allclose(supplied, polygon, atol=1e-7, rtol=0):
            raise ValueError("invalid_tray:observation.polygon_mismatch")
    t["polygon_xy"] = polygon
    if "observed_polygon_xy" in t:
        t["observed_polygon_xy"] = _polygon(t["observed_polygon_xy"], "observation.observed_polygon_xy")
    return t


def validate_objects(objects, config):
    if not isinstance(objects, list):
        raise ValueError("invalid_tray:objects")
    result, ids = [], set()
    for obj in objects:
        if not isinstance(obj, dict):
            raise ValueError("invalid_tray:object")
        obj = deepcopy(obj)
        key = obj.get("object_id")
        if type(key) is not int or key < 0 or key in ids:
            raise ValueError("invalid_tray:object_id")
        ids.add(key)
        if obj.get("frame_id") != config["frame_id"]:
            raise ValueError("invalid_tray:object.frame_id")
        if type(obj.get("position_valid")) is not bool:
            raise ValueError("invalid_tray:object.position_valid")
        if obj["position_valid"] or obj.get("center_xy") is not None:
            obj["center_xy"] = _vector(obj.get("center_xy"), 2, "object.center_xy")
        if obj["position_valid"] or obj.get("radius") is not None:
            obj["radius"] = _number(obj.get("radius"), "object.radius", positive=True)
            if obj["radius"] < config["bowl_radius"]:
                raise ValueError("invalid_tray:object.radius_too_small")
        result.append(obj)
    return result


def _rigid_matrix(transform):
    value = np.asarray(transform, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("invalid_tray:camera_to_base")
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5)
            or not math.isclose(np.linalg.det(value[:3, :3]), 1, abs_tol=1e-5)):
        raise ValueError("invalid_tray:camera_to_base:not_rigid")
    return value


def estimate_object_position(points_camera, camera_to_base, config):
    """Use the observed footprint plus the measured bowl radius and tolerance.

    Sparse/inconsistent depths stay unresolved; a rim candidate is never used
    as a surrogate bowl centre. Visible-point bounds are intentionally
    conservative and can suppress placements rather than underestimate space.
    """
    invalid = {"center_xy": None, "radius": None, "position_valid": False,
               "frame_id": config["frame_id"]}
    try:
        transform = _rigid_matrix(camera_to_base)
        points = np.asarray(points_camera, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("object_points_shape")
        total_points = len(points)
        points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
        if len(points) < 10:
            raise ValueError("insufficient_object_depth")
        if len(points) / total_points < .35:
            raise ValueError("insufficient_object_depth_coverage")
        world = points @ transform[:3, :3].T + transform[:3, 3]
        if config.get("object_geometry") == "visual_rim":
            from .visual_rim import estimate_visual_rim
            return {**estimate_visual_rim(world, config), "position_valid": True,
                    "frame_id": config["frame_id"], "position_reason": "visual_rim_circle"}
        if config.get("known_bowl_radii"):
            from rim_locator.config import RimConfig
            from rim_locator.rim_detection import locate_rim
            from rim_locator.circle_fit import fit_measured_rim_circle
            rim = locate_rim(world, RimConfig())
            fitted = fit_measured_rim_circle(rim.candidates, config["known_bowl_radii"])
            return {**fitted, "radius": fitted["measured_radius"] + config["position_tolerance"],
                    "center_z": rim.z_top, "position_valid": True, "frame_id": config["frame_id"],
                    "position_reason": "measured_rim_circle"}
        # Enclose ALL surface points. The midpoint of an axis-aligned bounding
        # box can inflate the disk and shift its centre for an oblique view.
        # This changes no depth samples and does not discard outliers to fit.
        import cv2
        center, _ = cv2.minEnclosingCircle(world[:, :2].astype(np.float32))
        center = np.asarray(center, dtype=float)
        observed_radius = float(np.max(np.linalg.norm(world[:, :2] - center, axis=1)))
        if observed_radius < .4 * config["bowl_radius"]:
            raise ValueError("insufficient_object_footprint")
        if observed_radius > 3 * config["bowl_radius"]:
            raise ValueError("inconsistent_object_footprint")
        radius = max(config["bowl_radius"], observed_radius) + config["position_tolerance"]
        return {"center_xy": center.tolist(), "radius": radius, "position_valid": True,
                "frame_id": config["frame_id"], "position_reason": "rgbd_footprint"}
    except (ValueError, TypeError) as exc:
        return {**invalid, "position_reason": str(exc)}


def estimate_obstacle_position(points_camera, camera_to_base, config):
    """Conservative observed footprint for non-grasp classes, never a rim pose.

    Uses every valid depth point; no guessed size or missing-depth fallback.
    Sparse or noisy evidence prevents placement instead of declaring space free.
    """
    invalid = dict(position_valid=False, frame_id=config['frame_id'], center_xy=None, radius=None)
    try:
        transform = _rigid_matrix(camera_to_base)
        points = np.asarray(points_camera, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('obstacle_points_shape')
        valid = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
        if len(valid) < 10 or len(valid) / max(1, len(points)) < .7:
            raise ValueError('insufficient_obstacle_depth')
        world = valid @ transform[:3, :3].T + transform[:3, 3]
        import cv2
        center, _ = cv2.minEnclosingCircle(world[:, :2].astype(np.float32))
        radius = float(np.max(np.linalg.norm(world[:, :2]-center, axis=1)))
        if not .001 <= radius <= .30:
            raise ValueError('inconsistent_obstacle_footprint')
        return dict(position_valid=True, frame_id=config['frame_id'], center_xy=list(center),
                    radius=radius+config['position_tolerance'], position_reason='observed_obstacle_footprint')
    except (ValueError, TypeError) as exc:
        return dict(invalid, position_reason=str(exc))


def _inside_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            if x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
                inside = not inside
    return inside


def _disk_inside_polygon(center, radius, polygon):
    return (_inside_polygon(center, polygon)
            and min(_segment_distance(center, a, b)
                    for a, b in zip(polygon, polygon[1:] + polygon[:1])) >= radius - 1e-9)


def classify_object(obj, tray, config, *, observed_occupancy=False):
    """Classify a footprint, or its center when registering observed occupancy.

    Center membership records an obstacle, not successful placement. Keep its
    full radius for avoidance; new destinations still require the whole disk
    to fit in enumerate_safe_slots/enumerate_grid_slots.
    """
    obj = validate_objects([obj], config)[0]
    tray = validate_tray_observation(tray, config)
    if tray["status"] != "ok" or not obj["position_valid"]:
        return "unknown"
    offset = np.asarray(obj["center_xy"]) - tray["center_xy"]
    local = np.asarray(rotate_xy(offset, -tray["yaw"]))
    half = np.asarray(tray["size"]) / 2
    radius = obj["radius"]
    containment_radius = 0. if observed_occupancy else radius
    if np.all(np.abs(local) + containment_radius <= half):
        if ("observed_polygon_xy" in tray
                and not _disk_inside_polygon(obj["center_xy"], containment_radius, tray["observed_polygon_xy"])):
            return "unknown"
        return "inside"
    if np.linalg.norm(np.maximum(np.abs(local) - half, 0)) <= radius:
        return "unknown"
    source = config.get("source_polygon")
    if source is not None:
        distance = min(_segment_distance(obj["center_xy"], a, b)
                       for a, b in zip(source, source[1:] + source[:1]))
        if distance <= radius:
            return "unknown"
        if not _inside_polygon(obj["center_xy"], source):
            return "out_of_source"
    return "outside"


def slot_xy(tray, slot, config):
    tray = validate_tray_observation(tray, config)
    if tray["status"] != "ok":
        raise ValueError("tray_not_localized")
    if config.get("placement_mode") == "grid":
        if not isinstance(slot, dict):
            raise ValueError("invalid_tray:grid_slot")
        uv = _vector(slot.get("uv"), 2, "grid_slot.uv")
        if slot.get("id") != "grid:" + ":".join(f"{v:.6f}" for v in uv):
            raise ValueError("invalid_tray:grid_slot_id")
        if any(abs(v) > size / 2 for v, size in zip(uv, tray["size"])):
            raise ValueError("invalid_tray:grid_slot_bounds")
        return (np.asarray(tray["center_xy"]) + rotate_xy(uv, tray["yaw"])).tolist()
    slot_id = slot if isinstance(slot, str) else slot.get("id")
    matches = [s for s in config["slots"] if s["id"] == slot_id]
    if len(matches) != 1:
        raise ValueError("invalid_tray:slot_id")
    if isinstance(slot, dict) and "uv" in slot and not np.allclose(slot["uv"], matches[0]["uv"]):
        raise ValueError("invalid_tray:slot_mismatch")
    return (np.asarray(tray["center_xy"]) + rotate_xy(matches[0]["uv"], tray["yaw"])).tolist()


def enumerate_safe_slots(tray, objects, used_slots, config, *, incoming_radius=None, remaining_radii=()):
    """Return all safe destination references in deterministic preference order.

    The returned values are references into the current tray observation. They
    are not world-coordinate commands and must be revalidated when selected.
    """
    tray = validate_tray_observation(tray, config)
    objects = validate_objects(objects, config)
    if not isinstance(used_slots, (set, list, tuple, frozenset)):
        raise ValueError("invalid_tray:used_slots")
    if config.get("placement_mode") == "grid":
        from .placement_grid import enumerate_grid_slots
        return enumerate_grid_slots(tray, objects, used_slots, config,
                                    incoming_radius=incoming_radius,
                                    remaining_radii=remaining_radii)
    valid_ids = {s["id"] for s in config["slots"]}
    if any(not isinstance(s, str) or s not in valid_ids for s in used_slots):
        raise ValueError("invalid_tray:used_slot_id")
    if tray["status"] != "ok" or any(not obj["position_valid"] for obj in objects):
        return []
    reserved_radius = config["bowl_radius"] + config["position_tolerance"]
    radius = reserved_radius
    if incoming_radius is not None:
        radius = max(radius, _number(incoming_radius, "incoming_radius", positive=True))
    reserved_xy = [np.asarray(slot_xy(tray, key, config)) for key in used_slots]
    results = []
    for slot in config["slots"]:
        if slot["id"] in used_slots:
            continue
        if any(abs(v) + radius + config["edge_margin"] > size / 2 + 1e-9
               for v, size in zip(slot["uv"], tray["size"])):
            continue
        xy = slot_xy(tray, slot, config)
        if ("observed_polygon_xy" in tray
                and not _disk_inside_polygon(xy, radius + config["edge_margin"], tray["observed_polygon_xy"])):
            continue
        if any(np.linalg.norm(np.asarray(xy) - previous) <= radius + reserved_radius + config["clearance"]
               for previous in reserved_xy):
            continue
        if any(np.linalg.norm(np.asarray(xy) - obj["center_xy"])
               <= radius + obj["radius"] + config["clearance"] for obj in objects):
            continue
        results.append({"id": slot["id"], "uv": list(slot["uv"]), "xy": xy})
    return results


def choose_slot(tray, objects, used_slots, config, *, incoming_radius=None, remaining_radii=()):
    """Return the first safe {id,uv,xy}, or None. Used IDs are never freed here."""
    slots = enumerate_safe_slots(tray, objects, used_slots, config,
                                 incoming_radius=incoming_radius,
                                 remaining_radii=remaining_radii)
    return slots[0] if slots else None


def _refine_boundary(bgr, mask):
    """Refine only the neighbourhood of an existing YOLO mask, never discover a tray.

    Interior YOLO pixels are fixed foreground; far exterior pixels are fixed
    background. Excessive changes fail rather than replace YOLO with a colour
    detector. The original image and mask remain untouched.
    """
    import cv2
    rows, cols = np.nonzero(mask)
    radius = max(3, min(10, int(min(np.ptp(rows), np.ptp(cols)) * .1)))
    pad = 2 * radius
    x0, x1 = max(0, cols.min() - pad), min(mask.shape[1], cols.max() + pad + 1)
    y0, y1 = max(0, rows.min() - pad), min(mask.shape[0], rows.max() + pad + 1)
    local = mask[y0:y1, x0:x1].astype(np.uint8)
    kernel = np.ones((2 * radius + 1, 2 * radius + 1), np.uint8)
    interior = cv2.erode(local, kernel)
    if np.count_nonzero(interior) < 100:
        raise ValueError("tray_boundary_refinement_insufficient_seed")
    labels = np.full(local.shape, cv2.GC_BGD, np.uint8)
    labels[cv2.dilate(local, kernel) > 0] = cv2.GC_PR_BGD
    labels[local > 0] = cv2.GC_PR_FGD
    labels[interior > 0] = cv2.GC_FGD
    cv2.grabCut(np.ascontiguousarray(bgr[y0:y1, x0:x1]), labels, None,
                np.zeros((1, 65)), np.zeros((1, 65)), 5, cv2.GC_INIT_WITH_MASK)
    refined = np.zeros_like(mask)
    refined[y0:y1, x0:x1] = (labels == cv2.GC_FGD) | (labels == cv2.GC_PR_FGD)
    if not .75 <= np.count_nonzero(refined) / np.count_nonzero(mask) <= 1.25:
        raise ValueError("tray_boundary_refinement_excessive_change")
    return refined


def _mask_plane(mask, xyz, transform, xy_tolerance=.006):
    """Fit a supported plane and pixel-to-base-XY mapping from this frame only."""
    import cv2
    rows, cols = np.nonzero(mask)
    points = xyz[rows, cols]
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
    if np.count_nonzero(valid) < 80 or np.mean(valid) < .70:
        raise ValueError("tray_insufficient_depth")
    pixels = np.column_stack((cols[valid], rows[valid])).astype(float)
    world = points[valid] @ transform[:3, :3].T + transform[:3, 3]
    # Deterministic sampling bounds cost without depending on camera resolution.
    step = max(1, math.ceil(len(world) / 4000))
    world, pixels = world[::step], pixels[::step]
    design = np.column_stack((world[:, :2], np.ones(len(world))))
    inliers = np.ones(len(world), dtype=bool)
    for _ in range(3):
        coefficients, _, rank, _ = np.linalg.lstsq(design[inliers], world[inliers, 2], rcond=None)
        if rank != 3:
            raise ValueError("tray_degenerate_depth")
        residual = np.abs(design @ coefficients - world[:, 2])
        inliers = residual <= .010
        if np.count_nonzero(inliers) < 80 or np.mean(inliers) < .70:
            raise ValueError("tray_inconsistent_depth")
    if np.linalg.norm(coefficients[:2]) > math.tan(math.radians(15)):
        raise ValueError("tray_not_horizontal")
    # A projective map is necessary for an oblique camera. It also allows
    # visibility checks without inventing camera intrinsics or table depth.
    mapping, _ = cv2.findHomography(pixels[inliers], world[inliers, :2], method=0)
    if mapping is None or not np.isfinite(mapping).all():
        raise ValueError("tray_degenerate_depth")
    projected = cv2.perspectiveTransform(pixels[inliers][None], mapping)[0]
    xy_residual = float(np.quantile(np.linalg.norm(projected - world[inliers, :2], axis=1), .95))
    if xy_residual > xy_tolerance:
        raise ValueError("tray_inconsistent_depth")
    depth_hull = cv2.convexHull(pixels[inliers].astype(np.float32))
    mask_hull = cv2.convexHull(np.column_stack((cols, rows)).astype(np.float32))
    if cv2.contourArea(depth_hull) < .70 * cv2.contourArea(mask_hull):
        raise ValueError("tray_insufficient_depth_extent")
    return mapping, int(np.count_nonzero(inliers)), xy_residual


def _project_region(polygon, mapping, bounds, name):
    points = np.column_stack((polygon, np.ones(len(polygon)))) @ mapping.T
    # The caller normalizes the denominator at the visible tray center.
    # Negative denominators are behind the camera, even if pixels look valid.
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 1e-10):
        raise ValueError(name + "_outside_detection_view")
    pixels = points[:, :2] / points[:, 2, None]
    x0, y0, x1, y1 = bounds
    if (np.any(pixels < [x0 + 2, y0 + 2])
            or np.any(pixels > [x1 - 2, y1 - 2])):
        raise ValueError(name + "_outside_detection_view")
    return pixels.tolist()


def detect_tray(bgr, xyz, config, camera_matrix=None, camera_to_base=None,
                inference_roi=None, *, detections=None, occluder_masks=()):
    """Localize exactly one YOLO-segmented rectangular tray in base XY.

    A bounding box alone is insufficient: the full instance mask, consistent
    same-frame depth, and measured outer dimensions must support the pose.
    The returned polygon always uses the measured usable interior. Occluded
    corners, truncated masks, ambiguous instances, and uncertain yaw fail
    closed. No previous pose or guessed depth is substituted.
    """
    import cv2
    invalid = {"status": "invalid", "frame_id": config["frame_id"]}
    try:
        transform = _rigid_matrix(camera_to_base)
        bgr, xyz = np.asarray(bgr), np.asarray(xyz, dtype=float)
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("tray_image_format")
        if xyz.shape != (*bgr.shape[:2], 3):
            raise ValueError("tray_depth_dimension_mismatch")
        if camera_matrix is not None:
            matrix = np.asarray(camera_matrix, dtype=float)
            if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
                raise ValueError("tray_invalid_intrinsics")
        if not isinstance(detections, (list, tuple)):
            raise ValueError("tray_yolo_detections_missing")
        matches = []
        for candidate in detections:
            if candidate.class_name != config["yolo"]["target_class"]:
                continue
            score = _number(candidate.confidence, "yolo_detection.confidence", nonnegative=True)
            if score > 1:
                raise ValueError("tray_invalid_confidence")
            if score >= config["yolo"]["confidence"]:
                matches.append(candidate)
        if not matches:
            return {"status": "not_found", "frame_id": config["frame_id"], "reason": "tray_yolo_not_found"}
        if len(matches) != 1:
            raise ValueError("tray_yolo_ambiguous")
        mask = np.asarray(matches[0].mask)
        if mask.shape != bgr.shape[:2] or mask.dtype != np.bool_ or np.count_nonzero(mask) < 100:
            raise ValueError("tray_mask_missing_or_invalid")
        rows, cols = np.nonzero(mask)
        height, width = mask.shape
        if rows.min() < 2 or cols.min() < 2 or rows.max() >= height - 2 or cols.max() >= width - 2:
            raise ValueError("tray_mask_truncated")
        if config.get("refine_boundary", False):
            mask = _refine_boundary(bgr, mask)
            rows, cols = np.nonzero(mask)
            if rows.min() < 2 or cols.min() < 2 or rows.max() >= height - 2 or cols.max() >= width - 2:
                raise ValueError("tray_mask_truncated")
        occluders = np.zeros(mask.shape, dtype=bool)
        for occluder in occluder_masks:
            if (not isinstance(occluder, np.ndarray) or occluder.dtype != np.bool_
                    or occluder.shape != mask.shape):
                raise ValueError("tray_invalid_occluder_mask")
            occluders |= occluder
        # Bowl pixels are above the tray plane. They cannot contribute depth
        # or protrude beyond its contour when the tray already contains bowls.
        mask = mask & ~occluders
        if np.count_nonzero(mask) < 100:
            raise ValueError("tray_insufficient_visible_surface")
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if sum(cv2.contourArea(c) for c in contours[1:]) > .02 * cv2.contourArea(contours[0]):
            raise ValueError("tray_mask_disconnected")
        mapping, depth_samples, xy_residual = _mask_plane(
            mask, xyz, transform, config.get("depth_xy_tolerance", .006))
        contour_xy = cv2.perspectiveTransform(contours[0].astype(np.float32), mapping)
        rectangle = cv2.minAreaRect(contour_xy)
        corners = cv2.boxPoints(rectangle).astype(float)
        edges = np.roll(corners, -1, axis=0) - corners
        lengths = np.linalg.norm(edges, axis=1)
        long_index = int(np.argmax(lengths))
        observed_size = np.array([lengths[long_index], lengths[(long_index + 1) % 4]])
        if min(observed_size) <= 0 or observed_size[0] / observed_size[1] < 1.12:
            raise ValueError("tray_yaw_ambiguous")
        outer_size = np.asarray(config["tray_outer_size"])
        if np.any(np.abs(observed_size - outer_size) > np.maximum(.008, .05 * outer_size)):
            raise ValueError("tray_outer_size_mismatch")
        axis = edges[long_index] / observed_size[0]
        yaw = (math.atan2(axis[1], axis[0]) + math.pi / 2) % math.pi - math.pi / 2
        center_xy = np.asarray(rectangle[0], dtype=float)
        hull = cv2.convexHull(contour_xy)
        measured_corners = tray_polygon(center_xy, yaw, outer_size)
        # Account for bounded measured projection noise in the corner test.
        # Placement disks must still fit the unexpanded observed hull.
        corner_tolerance = .07 * min(outer_size) + min(xy_residual, .008)
        visible_corners = sum(abs(cv2.pointPolygonTest(hull, tuple(corner), True))
                              <= corner_tolerance for corner in measured_corners)
        supported_by_occlusion = False
        if visible_corners < 4 and occluders.any():
            # Recover only pixels hidden by same-frame YOLO bowl instances,
            # inside the measured rectangle fitted from visible tray edges.
            # Missing corners without an observed occluder stay unsupported.
            inverse = np.linalg.inv(mapping)
            projected = cv2.perspectiveTransform(np.asarray(measured_corners)[None], inverse)[0]
            if not np.isfinite(projected).all() or np.max(np.abs(projected)) > 1e6:
                raise ValueError("tray_invalid_occlusion_projection")
            interior = np.zeros(mask.shape, np.uint8)
            cv2.fillConvexPoly(interior, np.rint(projected).astype(np.int32), 1)
            hidden = occluders & interior.astype(bool)
            if (visible_corners < 2 or not interior.any()
                    or hidden.sum() > .65 * interior.sum()):
                raise ValueError("tray_insufficient_visible_outline")
            hidden_rows, hidden_cols = np.nonzero(hidden)
            if len(hidden_rows):
                pixels = np.column_stack((hidden_cols, hidden_rows)).astype(np.float32)
                hidden_xy = cv2.perspectiveTransform(pixels[None], mapping)[0]
                hull = cv2.convexHull(np.concatenate((contour_xy[:, 0], hidden_xy)).astype(np.float32))
                supported_by_occlusion = True
        # Corners must be observed, not merely inferred from a large rectangle
        # around a partial contour. Small rounded tray corners remain allowed.
        if any(abs(cv2.pointPolygonTest(hull, tuple(corner), True)) > corner_tolerance
               for corner in measured_corners):
            raise ValueError("tray_outline_incomplete")
        if cv2.contourArea(hull) < .85 * float(np.prod(observed_size)):
            raise ValueError("tray_outline_incomplete")
        result = validate_tray_observation({
            "status": "ok", "frame_id": config["frame_id"],
            "center_xy": center_xy.tolist(), "yaw": yaw, "size": list(config["tray_size"]),
            "identity": "yolo:" + config["yolo"]["target_class"],
            "yolo_confidence": float(matches[0].confidence), "depth_samples": depth_samples,
            "depth_xy_residual_p95": xy_residual,
            "outline_corner_tolerance": corner_tolerance,
            "observed_outer_size": observed_size.tolist(), "yaw_modulo_pi": True,
            "observed_polygon_xy": hull[:, 0, :].tolist(),
            "boundary_refined": config.get("refine_boundary", False),
            "outline_supported_by_bowl_occlusion": supported_by_occlusion,
        }, config)
        # A rounded tray need not contain every corner of the nominal inner
        # rectangle. Each actual bowl/placement disk must fit the observed
        # hull as well as the measured interior (classify_object/choose_slot).
        bounds = [0., 0., float(width), float(height)] if inference_roi is None else _vector(
            inference_roi, 4, "inference_roi")
        x0, y0, x1, y1 = bounds
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise ValueError("tray_invalid_detection_view")
        inverse_mapping = np.linalg.inv(mapping)
        inverse_mapping /= inverse_mapping[2] @ np.r_[center_xy, 1.]
        result["base_xy_to_pixel"] = inverse_mapping.tolist()
        result["tray_outer_polygon_px"] = _project_region(corners, inverse_mapping, bounds, "tray")
        for name, polygon in (("tray", result["polygon_xy"]), ("source", config.get("source_polygon"))):
            if polygon is not None:
                result[name + "_polygon_px"] = _project_region(polygon, inverse_mapping, bounds, name)
        return result
    except (ValueError, TypeError, AttributeError, IndexError, np.linalg.LinAlgError, cv2.error) as exc:
        return {**invalid, "reason": str(exc)}
