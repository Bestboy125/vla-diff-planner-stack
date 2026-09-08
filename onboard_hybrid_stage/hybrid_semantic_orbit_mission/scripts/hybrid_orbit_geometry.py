"""Pure validation and staged-approach geometry for hybrid semantic orbiting."""
import math
import re


class HybridOrbitError(ValueError):
    pass


def validate_request(payload):
    if not isinstance(payload, dict):
        raise HybridOrbitError("request must be an object")
    task_id = payload.get("task_id")
    label = payload.get("target_label")
    if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
        raise HybridOrbitError("task_id is invalid")
    if not isinstance(label, str) or re.fullmatch(r"[A-Za-z][A-Za-z-]{0,31}", label) is None:
        raise HybridOrbitError("target_label must be one English word")
    try:
        radius = float(payload.get("radius_m"))
        laps = float(payload.get("laps"))
        baseline = float(payload.get("baseline_distance_m"))
    except (TypeError, ValueError) as exc:
        raise HybridOrbitError("radius, laps and baseline must be numbers") from exc
    if not math.isfinite(radius) or abs(radius - 1.5) > 1e-6:
        raise HybridOrbitError("radius_m must be exactly 1.5")
    if not math.isfinite(laps) or abs(laps - 1.0) > 1e-6:
        raise HybridOrbitError("laps must be exactly 1")
    if not math.isfinite(baseline) or not 0.5 <= baseline <= 1.0:
        raise HybridOrbitError("baseline_distance_m must be within [0.5, 1.0]")
    if payload.get("baseline_direction") not in ("left", "right"):
        raise HybridOrbitError("baseline_direction must be left or right")
    if payload.get("direction") != "clockwise":
        raise HybridOrbitError("hybrid orbit direction must be clockwise")
    if payload.get("yaw_mode") != "face_center":
        raise HybridOrbitError("yaw_mode must be face_center")
    if payload.get("keep_current_altitude") is not True:
        raise HybridOrbitError("keep_current_altitude must be true")
    return {
        "task_id": task_id,
        "target_label": label.lower(),
        "radius_m": radius,
        "laps": laps,
        "direction": "clockwise",
        "baseline_distance_m": baseline,
        "baseline_direction": payload["baseline_direction"],
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
    }


def next_handoff_leg(current, target, handoff_distance_m, max_leg_m):
    x, y, z = [float(value) for value in current]
    tx, ty, _tz = [float(value) for value in target]
    handoff = float(handoff_distance_m)
    max_leg = float(max_leg_m)
    if not all(math.isfinite(value) for value in (x, y, z, tx, ty, handoff, max_leg)):
        raise HybridOrbitError("approach geometry contains non-finite values")
    if handoff <= 0.0 or max_leg <= 0.0:
        raise HybridOrbitError("handoff distance and maximum leg must be positive")
    distance = math.hypot(tx - x, ty - y)
    remaining = max(0.0, distance - handoff)
    if remaining <= 1e-6:
        return None, distance
    leg = min(max_leg, remaining)
    waypoint = (x + (tx - x) * leg / distance, y + (ty - y) * leg / distance, z)
    return waypoint, distance
