#!/usr/bin/env python3
import math

from semantic_orbit_contract import (SemanticOrbitError, altitude_agl,
                                     build_staged_orbit_spec, build_world_orbit_spec,
                                     validate_semantic_orbit_request)


def main():
    request = validate_semantic_orbit_request({
        "task_id": "dry-run",
        "target_label": "Chair",
        "radius_m": 1.5,
        "laps": 1,
        "direction": "clockwise",
        "yaw_mode": "face_center",
        "keep_current_altitude": True,
    })
    spec = build_world_orbit_spec(request, (2.0, 0.0, 0.7), (0.0, 0.0, 1.2), 2.0)
    assert request["target_label"] == "chair"
    assert spec["center"] == (2.0, 0.0, 1.2)
    assert spec["entry_world"] == (0.5, 0.0, 1.2)
    assert abs(math.hypot(spec["entry_world"][0] - spec["center"][0],
                          spec["entry_world"][1] - spec["center"][1]) - 1.5) < 1e-9
    assert abs(spec["approach_leg_m"] - 0.5) < 1e-9
    assert spec["approach_required"] is False
    assert spec["direction"] == "cw"
    assert abs(spec["orbit_angle_rad"] - 2.0 * math.pi) < 1e-9
    assert abs(altitude_agl(0.0, -0.8, 0.3, 2.0) - 0.8) < 1e-9
    try:
        altitude_agl(-0.7, -0.8, 0.3, 2.0)
    except SemanticOrbitError:
        pass
    else:
        raise AssertionError("below-minimum AGL was not rejected")
    try:
        build_world_orbit_spec(request, (6.0, 0.0, 0.7), (0.0, 0.0, 1.2), 2.0)
    except SemanticOrbitError:
        pass
    else:
        raise AssertionError("long approach leg was not rejected")
    staged = build_staged_orbit_spec(
        request, (6.0, 0.0, 0.7), (0.0, 0.0, 1.2), 2.0
    )
    assert staged["approach_required"] is True
    assert staged["entry_world"] == (4.5, 0.0, 1.2)
    assert staged["approach_waypoint_world"] == (2.0, 0.0, 1.2)
    assert abs(staged["approach_leg_m"] - 4.5) < 1e-9
    print("semantic orbit contract self-test passed; no ROS topics were opened")


if __name__ == "__main__":
    main()
