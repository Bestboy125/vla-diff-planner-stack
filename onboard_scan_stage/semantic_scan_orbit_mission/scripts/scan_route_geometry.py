#!/usr/bin/env python3
"""Pure geometry and contract validation for the fixed serpentine scan mission."""
import math
import re


class ScanMissionError(ValueError):
    pass


def _finite_float(payload, key):
    try:
        value = float(payload.get(key))
    except (TypeError, ValueError) as exc:
        raise ScanMissionError("%s must be a finite number" % key) from exc
    if not math.isfinite(value):
        raise ScanMissionError("%s must be a finite number" % key)
    return value


def validate_request(payload):
    if not isinstance(payload, dict):
        raise ScanMissionError("request must be an object")
    task_id = payload.get("task_id")
    if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
        raise ScanMissionError("task_id is invalid")
    target_label = payload.get("target_label")
    if not isinstance(target_label, str) or re.fullmatch(r"[A-Za-z][A-Za-z-]{0,31}", target_label) is None:
        raise ScanMissionError("target_label must be one English word")
    width = _finite_float(payload, "scan_width_m")
    length = _finite_float(payload, "scan_length_m")
    radius = _finite_float(payload, "radius_m")
    laps = _finite_float(payload, "laps")
    sweeps = payload.get("sweep_count")
    if abs(width - 6.0) > 1e-6 or abs(length - 10.0) > 1e-6:
        raise ScanMissionError("scan preset must be exactly 6 m x 10 m")
    if isinstance(sweeps, bool) or sweeps != 5:
        raise ScanMissionError("scan preset must contain exactly 5 sweeps")
    if target_label.lower() != "chair":
        raise ScanMissionError("scan interrupt target must be chair")
    if abs(radius - 1.5) > 1e-6 or abs(laps - 1.0) > 1e-6:
        raise ScanMissionError("chair orbit must be radius 1.5 m and one lap")
    if payload.get("direction") != "clockwise":
        raise ScanMissionError("chair orbit must be clockwise")
    if payload.get("yaw_mode") != "face_center":
        raise ScanMissionError("chair orbit must face the target center")
    if payload.get("keep_current_altitude") is not True:
        raise ScanMissionError("scan mission must keep current altitude")
    return {
        "task_id": task_id,
        "target_label": "chair",
        "scan_width_m": 6.0,
        "scan_length_m": 10.0,
        "sweep_count": 5,
        "radius_m": 1.5,
        "laps": 1.0,
        "direction": "clockwise",
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
    }


def generate_local_serpentine(width_m=6.0, length_m=10.0, sweep_count=5,
                              waypoint_spacing_m=1.0, turn_samples=6,
                              turn_bulge_ratio=0.45):
    """World-aligned offsets: 6 m sweeps along X, 10 m expansion along +Y.

    Legacy wire names width_m/length_m denote the X/Y extents respectively.
    These offsets are translated by the task origin, never rotated by body yaw.
    """
    width = float(width_m)
    length = float(length_m)
    sweeps = int(sweep_count)
    spacing = float(waypoint_spacing_m)
    samples = int(turn_samples)
    bulge_ratio = float(turn_bulge_ratio)
    if (not all(math.isfinite(v) for v in (width, length, spacing, bulge_ratio))
            or width <= 0.0 or length <= 0.0 or sweeps < 2 or spacing <= 0.0
            or samples < 2 or not 0.0 <= bulge_ratio <= 0.5):
        raise ScanMissionError("invalid serpentine geometry")
    row_spacing = length / float(sweeps - 1)
    points = []

    def append(x_value, y_value, kind, sweep_index):
        point = {
            "local": (float(x_value), float(y_value), 0.0),
            "kind": kind,
            "sweep_index": int(sweep_index),
        }
        if points:
            previous = points[-1]["local"]
            if math.hypot(point["local"][0] - previous[0], point["local"][1] - previous[1]) < 1e-9:
                return
        points.append(point)

    lane_steps = max(1, int(math.ceil(width / spacing)))
    for row in range(sweeps):
        y_value = row * row_spacing
        start_x, end_x = (0.0, width) if row % 2 == 0 else (width, 0.0)
        for index in range(lane_steps + 1):
            fraction = index / float(lane_steps)
            append(start_x + (end_x - start_x) * fraction, y_value, "scan", row)
        if row == sweeps - 1:
            continue
        # A sampled U-shaped connector remains inside the 6 x 10 m scan box.
        # Diff-Planner receives each sample independently and performs the
        # dynamically feasible trajectory optimization between them.
        inward = -1.0 if end_x > 0.5 * width else 1.0
        bulge = min(0.5 * width, bulge_ratio * row_spacing)
        for index in range(1, samples + 1):
            fraction = index / float(samples)
            connector_x = end_x + inward * bulge * math.sin(math.pi * fraction)
            connector_y = y_value + row_spacing * fraction
            append(connector_x, connector_y, "turn", row)
    return points


def local_to_world(local_point, origin_world):
    lx, ly, lz = [float(value) for value in local_point]
    ox, oy, oz = [float(value) for value in origin_world]
    if not all(math.isfinite(v) for v in (lx, ly, lz, ox, oy, oz)):
        raise ScanMissionError("route coordinates must be finite")
    return (ox + lx, oy + ly, oz + lz)


def generate_world_serpentine(origin_world, **kwargs):
    result = []
    for item in generate_local_serpentine(**kwargs):
        world_item = dict(item)
        world_item["world"] = local_to_world(item["local"], origin_world)
        if result:
            previous = result[-1]["world"]
            current = world_item["world"]
            world_item["yaw_rad"] = math.atan2(current[1] - previous[1],
                                               current[0] - previous[0])
        result.append(world_item)
    # The first generated point is exactly the capture pose and is not a goal.
    return result[1:]


def horizontal_distance(first, second):
    return math.hypot(float(first[0]) - float(second[0]),
                      float(first[1]) - float(second[1]))
