#!/usr/bin/env python3
import math
from hybrid_orbit_geometry import next_handoff_leg, validate_request


request = validate_request({
    "task_id": "dry-test", "target_label": "chair", "radius_m": 1.5,
    "laps": 1.0, "direction": "clockwise", "baseline_distance_m": 0.75,
    "baseline_direction": "right", "yaw_mode": "face_center",
    "keep_current_altitude": True,
})
assert request["target_label"] == "chair"
waypoint, distance = next_handoff_leg((0, 0, 1), (10, 0, 0), 4.0, 2.0)
assert math.isclose(distance, 10.0) and waypoint == (2.0, 0.0, 1.0)
waypoint, distance = next_handoff_leg((6, 0, 1), (10, 0, 0), 4.0, 2.0)
assert waypoint is None and math.isclose(distance, 4.0)
print("hybrid orbit geometry self-test passed; no ROS topics were opened")
