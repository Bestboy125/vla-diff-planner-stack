"""Pure validation and frame geometry for two-position monocular orbiting."""
import math
import re

import numpy as np


class MonocularOrbitError(ValueError):
    pass


def validate_body_t_camera(values):
    try:
        transform = np.asarray(values, dtype=np.float64).reshape(4, 4)
    except (TypeError, ValueError) as exc:
        raise MonocularOrbitError("body_T_camera must contain a 4x4 matrix") from exc
    if (not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)):
        raise MonocularOrbitError("body_T_camera must be a finite homogeneous transform")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=2e-3) or
            np.linalg.det(rotation) < 0.99):
        raise MonocularOrbitError("body_T_camera rotation must be right-handed and orthonormal")
    return transform


def validate_request(payload):
    if not isinstance(payload, dict):
        raise MonocularOrbitError("request must be an object")
    label = payload.get("target_label")
    if not isinstance(label, str) or re.fullmatch(r"[A-Za-z][A-Za-z-]{0,31}", label) is None:
        raise MonocularOrbitError("target_label must be one English word")
    try:
        radius = float(payload.get("radius_m"))
        laps = float(payload.get("laps"))
        baseline = float(payload.get("baseline_distance_m"))
    except (TypeError, ValueError) as exc:
        raise MonocularOrbitError("radius, laps and baseline must be numbers") from exc
    if not math.isfinite(radius) or abs(radius - 1.5) > 1e-6:
        raise MonocularOrbitError("radius_m must be exactly 1.5")
    if not math.isfinite(laps) or abs(laps - 1.0) > 1e-6:
        raise MonocularOrbitError("laps must be exactly 1")
    if not math.isfinite(baseline) or not 0.5 <= baseline <= 1.0:
        raise MonocularOrbitError("second-viewpoint displacement must be within [0.5, 1.0] m")
    direction = payload.get("direction")
    if direction not in ("clockwise", "counterclockwise"):
        raise MonocularOrbitError("direction is invalid")
    baseline_direction = payload.get("baseline_direction")
    if baseline_direction not in ("left", "right"):
        raise MonocularOrbitError("baseline_direction must be left or right")
    if payload.get("yaw_mode") != "face_center":
        raise MonocularOrbitError("yaw_mode must be face_center")
    if payload.get("keep_current_altitude") is not True:
        raise MonocularOrbitError("keep_current_altitude must be true")
    localize_only = payload.get("localize_only", False)
    if not isinstance(localize_only, bool):
        raise MonocularOrbitError("localize_only must be boolean")
    manage_stereo_inference = payload.get("manage_stereo_inference", True)
    if not isinstance(manage_stereo_inference, bool):
        raise MonocularOrbitError("manage_stereo_inference must be boolean")
    task_id = payload.get("task_id")
    if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
        raise MonocularOrbitError("task_id is invalid")
    return {
        "task_id": task_id,
        "target_label": label.lower(),
        "radius_m": radius,
        "laps": laps,
        "direction": direction,
        "baseline_distance_m": baseline,
        "baseline_direction": baseline_direction,
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
        "localize_only": localize_only,
        "manage_stereo_inference": manage_stereo_inference,
    }


def quaternion_matrix_xyzw(values):
    x, y, z, w = [float(item) for item in values]
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or norm < 1e-12:
        raise MonocularOrbitError("invalid pose quaternion")
    scale = 2.0 / norm
    return np.asarray([
        [1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y)],
    ], dtype=np.float64)


def camera_pose_world(body_position, body_quaternion, body_from_camera_translation,
                      body_from_camera_quaternion):
    position = np.asarray(body_position, dtype=np.float64).reshape(3)
    translation = np.asarray(body_from_camera_translation, dtype=np.float64).reshape(3)
    rotation_world_body = quaternion_matrix_xyzw(body_quaternion)
    rotation_value = np.asarray(body_from_camera_quaternion, dtype=np.float64)
    if rotation_value.shape == (3, 3):
        rotation_body_camera = rotation_value
        if (not np.isfinite(rotation_body_camera).all() or
                not np.allclose(rotation_body_camera.T.dot(rotation_body_camera),
                                np.eye(3), atol=2e-3) or
                np.linalg.det(rotation_body_camera) < 0.99):
            raise MonocularOrbitError("camera rotation matrix is invalid")
    else:
        rotation_body_camera = quaternion_matrix_xyzw(rotation_value.reshape(4))
    rotation_world_camera = rotation_world_body.dot(rotation_body_camera)
    camera_position_world = position + rotation_world_body.dot(translation)
    if not np.isfinite(camera_position_world).all():
        raise MonocularOrbitError("camera pose is non-finite")
    return camera_position_world, rotation_world_camera


def baseline_in_first_camera(camera_a_world, rotation_world_camera_a, camera_b_world):
    baseline = np.asarray(rotation_world_camera_a, dtype=np.float64).T.dot(
        np.asarray(camera_b_world, dtype=np.float64) - np.asarray(camera_a_world, dtype=np.float64)
    )
    if not np.isfinite(baseline).all():
        raise MonocularOrbitError("measured camera baseline is non-finite")
    return baseline


def baseline_waypoint(body_position, body_quaternion, distance_m, direction):
    distance = float(distance_m)
    if not math.isfinite(distance) or distance <= 0.0:
        raise MonocularOrbitError("baseline distance must be positive")
    sign = 1.0 if direction == "left" else -1.0 if direction == "right" else None
    if sign is None:
        raise MonocularOrbitError("baseline direction must be left or right")
    rotation_world_body = quaternion_matrix_xyzw(body_quaternion)
    offset = rotation_world_body.dot(np.asarray([0.0, sign * distance, 0.0]))
    start = np.asarray(body_position, dtype=np.float64)
    result = start + offset
    result[2] = start[2]
    return tuple(float(value) for value in result)


def target_world_from_camera(target_camera_b, camera_b_world, rotation_world_camera_b):
    target = np.asarray(camera_b_world, dtype=np.float64) + np.asarray(
        rotation_world_camera_b, dtype=np.float64
    ).dot(np.asarray(target_camera_b, dtype=np.float64))
    if target.shape != (3,) or not np.isfinite(target).all():
        raise MonocularOrbitError("world target is non-finite")
    return tuple(float(value) for value in target)


def bbox_iou_xywh_xyxy(bbox_xywh, bbox_xyxy):
    x, y, width, height = [float(value) for value in bbox_xywh]
    ax1, ay1, ax2, ay2 = x, y, x + width, y + height
    bx1, by1, bx2, by2 = [float(value) for value in bbox_xyxy]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, ix2-ix1) * max(0.0, iy2-iy1)
    union = max(0.0, ax2-ax1) * max(0.0, ay2-ay1) + max(0.0, bx2-bx1) * max(0.0, by2-by1) - intersection
    return intersection / union if union > 0.0 else 0.0


def build_orbit_step(request, target_world, current_position, max_leg_m):
    tx, ty, _ = [float(value) for value in target_world]
    x, y, z = [float(value) for value in current_position]
    distance = math.hypot(x-tx, y-ty)
    if distance <= 1e-6:
        raise MonocularOrbitError("circle entry is undefined at target center")
    radius = request["radius_m"]
    entry = (tx + radius*(x-tx)/distance, ty + radius*(y-ty)/distance, z)
    remaining = math.hypot(x-entry[0], y-entry[1])
    approach = remaining > float(max_leg_m)
    waypoint = entry
    if approach:
        ratio = float(max_leg_m) / remaining
        waypoint = (x + (entry[0]-x)*ratio, y + (entry[1]-y)*ratio, z)
    return {
        "center": (tx, ty, z),
        "entry_world": entry,
        "approach_required": approach,
        "approach_waypoint_world": waypoint,
        "remaining_to_entry_m": remaining,
        "direction": "cw" if request["direction"] == "clockwise" else "ccw",
        "orbit_angle_rad": 2.0 * math.pi * request["laps"],
    }
