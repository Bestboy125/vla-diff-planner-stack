"""Pure validation and geometry helpers for the onboard semantic-orbit workflow."""
import math
import re


class SemanticOrbitError(ValueError):
    pass


def validate_semantic_orbit_request(payload):
    if not isinstance(payload, dict):
        raise SemanticOrbitError("request must be an object")
    target = payload.get("target_label")
    if not isinstance(target, str) or re.fullmatch(r"[A-Za-z][A-Za-z-]{0,31}", target) is None:
        raise SemanticOrbitError("target_label must be one English word")
    try:
        radius = float(payload.get("radius_m"))
        laps = float(payload.get("laps"))
    except (TypeError, ValueError) as exc:
        raise SemanticOrbitError("radius_m and laps must be numbers") from exc
    if not math.isfinite(radius) or abs(radius - 1.5) > 1e-6:
        raise SemanticOrbitError("radius_m must be exactly 1.5")
    if not math.isfinite(laps) or abs(laps - 1.0) > 1e-6:
        raise SemanticOrbitError("laps must be exactly 1")
    direction = payload.get("direction")
    if direction not in ("clockwise", "counterclockwise"):
        raise SemanticOrbitError("direction is invalid")
    if payload.get("yaw_mode") != "face_center":
        raise SemanticOrbitError("yaw_mode must be face_center")
    if payload.get("keep_current_altitude") is not True:
        raise SemanticOrbitError("keep_current_altitude must be true")
    task_id = payload.get("task_id")
    if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
        raise SemanticOrbitError("task_id is invalid")
    return {
        "task_id": task_id,
        "target_label": target.lower(),
        "radius_m": radius,
        "laps": laps,
        "direction": direction,
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
    }


def build_world_orbit_spec(request, target_world, current_position, max_approach_leg_m):
    values = tuple(float(value) for value in tuple(target_world) + tuple(current_position))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise SemanticOrbitError("target and current position must contain three finite values")
    tx, ty, _target_z, x, y, z = values
    horizontal_distance = math.hypot(x - tx, y - ty)
    if horizontal_distance <= 1e-6:
        raise SemanticOrbitError("circle entry direction is undefined at the target center")
    # Both inputs are already in the odometry world frame.  The entry point is
    # therefore generated in that same frame on the ray from the target centre
    # towards the current vehicle position.  No body-frame delta is published.
    radius = request["radius_m"]
    entry_world = (
        tx + radius * (x - tx) / horizontal_distance,
        ty + radius * (y - ty) / horizontal_distance,
        z,
    )
    approach_leg = math.hypot(x - entry_world[0], y - entry_world[1])
    if approach_leg > float(max_approach_leg_m):
        raise SemanticOrbitError(
            "circle entry is %.3f m away, above the %.3f m approach limit"
            % (approach_leg, max_approach_leg_m)
        )
    return {
        "center": (tx, ty, z),
        "entry_world": entry_world,
        "radius_m": radius,
        "orbit_angle_rad": 2.0 * math.pi * request["laps"],
        "direction": "cw" if request["direction"] == "clockwise" else "ccw",
        "yaw_mode": request["yaw_mode"],
        "approach_leg_m": approach_leg,
    }
