#!/usr/bin/env python3
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from semantic_geometry import body_to_world, camera_to_body, camera_to_world, pinhole_point, stereo_point, standoff_goal


def close(a, b, eps=1e-9):
    return all(abs(x-y) < eps for x, y in zip(a, b))


p_cam = stereo_point((350, 200, 390, 240), 20, 500, 500, 320, 240, 0.12)
assert close(p_cam, (0.3, -0.12, 3.0)), p_cam
assert close(pinhole_point(370, 220, 3.0, 500, 500, 320, 240), p_cam)
r_body_camera = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))
p_world = camera_to_world(p_cam, (-5, 0, 1), (0, 0, 0, 1),
                          r_body_camera, (0, 0, 0))
assert close(p_world, (-2, -0.3, 1.12)), p_world
assert close(camera_to_body(p_cam, r_body_camera, (0, 0, 0)), (3.0, -0.3, 0.12))
assert close(body_to_world((3.0, -0.3, 0.12), (-5, 0, 1), (0, 0, 0, 1)), p_world)
goal = standoff_goal((-5, 0, 1), p_world, 1.0)
assert close(goal, (-2.995037190209989, -0.2004962809790011, 1.0)), goal
goal_at_target_height = standoff_goal((-5, 0, 1), p_world, 1.0, keep_body_altitude=False)
assert close(goal_at_target_height, (-2.995037190209989, -0.2004962809790011, 1.12)), goal_at_target_height
print("SEMANTIC_GEOMETRY_TEST_PASSED camera=%s world=%s goal=%s" % (p_cam, p_world, goal))
