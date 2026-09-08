#!/usr/bin/env python3
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from monocular_orbit_geometry import (
    baseline_in_first_camera, baseline_waypoint, bbox_iou_xywh_xyxy,
    build_orbit_step, camera_pose_world, target_world_from_camera,
    validate_body_t_camera, validate_request,
)


request = validate_request({
    "task_id": "self-test", "target_label": "chair", "radius_m": 1.5,
    "laps": 1, "direction": "clockwise", "baseline_distance_m": 0.6,
    "baseline_direction": "right", "yaw_mode": "face_center",
    "keep_current_altitude": True,
})
waypoint = baseline_waypoint((1, 2, 3), (0, 0, 0, 1), 0.6, "right")
np.testing.assert_allclose(waypoint, (1, 1.4, 3), atol=1e-9)
camera_a, rotation_a = camera_pose_world((0, 0, 1), (0, 0, 0, 1),
                                         (0.1, 0, 0), (0, 0, 0, 1))
camera_b, rotation_b = camera_pose_world((0, -0.6, 1), (0, 0, 0, 1),
                                         (0.1, 0, 0), (0, 0, 0, 1))
np.testing.assert_allclose(baseline_in_first_camera(camera_a, rotation_a, camera_b),
                           (0, -0.6, 0), atol=1e-9)
np.testing.assert_allclose(target_world_from_camera((0, 0, 4), camera_b, rotation_b),
                           (0.1, -0.6, 5), atol=1e-9)
body_t_camera = validate_body_t_camera([
    1, 0, 0, 0.1, 0, 1, 0, -0.2, 0, 0, 1, 0.3, 0, 0, 0, 1,
])
fixed_camera, fixed_rotation = camera_pose_world(
    (1, 2, 3), (0, 0, 0, 1), body_t_camera[:3, 3], body_t_camera[:3, :3]
)
np.testing.assert_allclose(fixed_camera, (1.1, 1.8, 3.3), atol=1e-9)
np.testing.assert_allclose(fixed_rotation, np.eye(3), atol=1e-9)
assert bbox_iou_xywh_xyxy((10, 10, 20, 20), (10, 10, 30, 30)) == 1.0
step = build_orbit_step(request, (6, 0, 1), (0, 0, 1), 2.0)
assert step["approach_required"]
np.testing.assert_allclose(step["approach_waypoint_world"], (2, 0, 1), atol=1e-9)
assert math.isclose(step["remaining_to_entry_m"], 4.5)
print("monocular orbit geometry self-test passed; no ROS topics were opened")
