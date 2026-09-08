"""Pure validation and geometry helpers for the onboard semantic-orbit workflow."""
import math
import re


class SemanticOrbitError(ValueError):
    pass


def altitude_agl(world_z_m, reference_z_m, minimum_agl_m, maximum_agl_m):
    values = tuple(float(value) for value in
                   (world_z_m, reference_z_m, minimum_agl_m, maximum_agl_m))
    if not all(math.isfinite(value) for value in values):
        raise SemanticOrbitError("altitude reference and bounds must be finite")
    world_z, reference_z, minimum_agl, maximum_agl = values
    if minimum_agl >= maximum_agl:
        raise SemanticOrbitError("minimum AGL must be below maximum AGL")
    agl_m = world_z - reference_z
    if not minimum_agl <= agl_m <= maximum_agl:
        raise SemanticOrbitError(
            "vehicle altitude %.3f m AGL is outside [%.3f, %.3f] m; "
            "world_z=%.3f, reference_z=%.3f" %
            (agl_m, minimum_agl, maximum_agl, world_z, reference_z)
        )
    return agl_m


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
    seed = payload.get("target_world_hint")
    if seed is not None:
        if not isinstance(seed, (list, tuple)) or len(seed) != 3:
            raise SemanticOrbitError("target_world_hint must contain XYZ")
        try:
            seed = tuple(float(v) for v in seed)
        except (TypeError, ValueError) as exc:
            raise SemanticOrbitError("target_world_hint must be finite") from exc
        if not all(math.isfinite(v) for v in seed):
            raise SemanticOrbitError("target_world_hint must be finite")
    return {
        "task_id": task_id,
        "target_world_hint": seed,
        "target_label": target.lower(),
        "radius_m": radius,
        "laps": laps,
        "direction": direction,
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
    }


def build_world_orbit_spec(request, target_world, current_position, max_approach_leg_m):
    spec = build_staged_orbit_spec(
        request, target_world, current_position, max_approach_leg_m
    )
    if spec["approach_required"]:
        raise SemanticOrbitError(
            "circle entry is %.3f m away, above the %.3f m approach limit"
            % (spec["approach_leg_m"], max_approach_leg_m)
        )
    return spec


def build_staged_orbit_spec(request, target_world, current_position,
                            max_approach_leg_m):
    values = tuple(float(value) for value in tuple(target_world) + tuple(current_position))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise SemanticOrbitError("target and current position must contain three finite values")
    max_approach_leg = float(max_approach_leg_m)
    if not math.isfinite(max_approach_leg) or max_approach_leg <= 0.0:
        raise SemanticOrbitError("maximum approach leg must be finite and positive")
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
    approach_required = approach_leg > max_approach_leg
    approach_waypoint = entry_world
    if approach_required:
        scale = max_approach_leg / approach_leg
        approach_waypoint = (
            x + (entry_world[0] - x) * scale,
            y + (entry_world[1] - y) * scale,
            z,
        )
    return {
        "center": (tx, ty, z),
        "entry_world": entry_world,
        "radius_m": radius,
        "orbit_angle_rad": 2.0 * math.pi * request["laps"],
        "direction": "cw" if request["direction"] == "clockwise" else "ccw",
        "yaw_mode": request["yaw_mode"],
        "approach_leg_m": approach_leg,
        "approach_required": approach_required,
        "approach_waypoint_world": approach_waypoint,
    }
