#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from scan_route_geometry import (generate_local_serpentine,
                                 generate_world_serpentine, validate_request)


request = validate_request({
    "task_id": "scan-test", "target_label": "chair",
    "scan_width_m": 6.0, "scan_length_m": 10.0, "sweep_count": 5,
    "radius_m": 1.5, "laps": 1.0, "direction": "clockwise",
    "yaw_mode": "face_center", "keep_current_altitude": True,
})
assert request["target_label"] == "chair"
points = generate_local_serpentine()
assert points[0]["local"] == (0.0, 0.0, 0.0)
assert points[-1]["local"] == (6.0, 10.0, 0.0)
scan_rows = sorted(set(item["sweep_index"] for item in points if item["kind"] == "scan"))
assert scan_rows == [0, 1, 2, 3, 4]
for item in points:
    x_value, y_value, z_value = item["local"]
    assert -1e-9 <= x_value <= 6.0 + 1e-9
    assert -1e-9 <= y_value <= 10.0 + 1e-9
    assert z_value == 0.0
for first, second in zip(points, points[1:]):
    distance = math.hypot(second["local"][0] - first["local"][0],
                          second["local"][1] - first["local"][1])
    assert distance <= 1.01
for row in range(5):
    lane = [p["local"] for p in points if p["kind"] == "scan" and p["sweep_index"] == row]
    assert all(math.isclose(p[1], row * 2.5) for p in lane)
    assert all((b[0] - a[0]) * (1 if row % 2 == 0 else -1) > 0
               for a, b in zip(lane, lane[1:]))
    assert math.isclose(lane[-1][0], 6.0 if row % 2 == 0 else 0.0)
world = generate_world_serpentine((10.0, 20.0, 1.2))
assert len(world) == len(points) - 1
assert world[0]["world"] == (11.0, 20.0, 1.2)
assert math.isclose(world[-1]["world"][0], 16.0, abs_tol=1e-9)
assert math.isclose(world[-1]["world"][1], 30.0, abs_tol=1e-9)
assert math.isclose(world[-1]["world"][2], 1.2, abs_tol=1e-9)
previous = (10.0, 20.0, 1.2)
for item in world:
    x, y, _ = item["world"]
    expected = math.atan2(y - previous[1], x - previous[0])
    assert abs(math.atan2(math.sin(item["yaw_rad"] - expected),
                          math.cos(item["yaw_rad"] - expected))) < 1e-9
    previous = item["world"]
assert world[0]["yaw_rad"] == 0.0
assert world[-1]["yaw_rad"] == 0.0
# Changing task origins translates the same world-aligned route, never rotates it.
translated = generate_world_serpentine((-2.0, 5.0, 0.8))
for first, second in zip(world, translated):
    for a, b, offset in zip(first["world"], second["world"], (12.0, 15.0, 0.4)):
        assert math.isclose(a - b, offset, abs_tol=1e-9)
    assert math.isclose(first["yaw_rad"], second["yaw_rad"], abs_tol=1e-9)
print("scan route geometry self-test passed; no ROS topics were opened")
