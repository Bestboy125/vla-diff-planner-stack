#!/usr/bin/env python3
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from raw_stereo_geometry import (bbox_center_point, body_to_world, camera_to_body,
                                 select_bbox_feature_cluster, triangulate_rectified)


fx = fy = 400.0
cx, cy = 320.0, 240.0
baseline = 0.05
left_p = np.array([[fx, 0, cx, 0], [0, fy, cy, 0], [0, 0, 1, 0]], dtype=float)
right_p = np.array([[fx, 0, cx, -fx*baseline], [0, fy, cy, 0], [0, 0, 1, 0]], dtype=float)
points_3d = np.array([[-0.2, -0.1, 2.0], [0.2, -0.1, 2.0], [-0.2, 0.1, 2.0], [0.2, 0.1, 2.0]])


def project(projection, points):
    homogeneous = np.column_stack([points, np.ones(len(points))])
    uvw = (projection @ homogeneous.T).T
    return uvw[:, :2] / uvw[:, 2:3]


left_uv = project(left_p, points_3d)
right_uv = project(right_p, points_3d)
result = triangulate_rectified(left_uv, right_uv, left_p, right_p)
assert abs(result["depth_m"] - 2.0) < 1e-8
assert abs(result["baseline_m"] - baseline) < 1e-8
center = bbox_center_point((280, 220, 360, 260), result["depth_m"], left_p)
np.testing.assert_allclose(center, [0, 0, 2], atol=1e-8)
cluster_points = np.asarray([[300, 220], [305, 222], [310, 225], [315, 230],
                             [500, 100], [540, 100]], dtype=float)
indices = select_bbox_feature_cluster(cluster_points, (280, 200, 360, 280), min_points=4)
np.testing.assert_array_equal(indices, [0, 1, 2, 3])

# Explicitly verify camera -> body -> odometry-world composition.  A camera
# point one metre forward is translated 0.2 m in body X, then a 90-degree body
# yaw maps it to world +Y at the current world position.
body_t_camera = np.eye(4)
body_t_camera[0, 3] = 0.2
point_body = camera_to_body(np.asarray([1.0, 0.0, 0.0]), body_t_camera)
half_sqrt_two = np.sqrt(0.5)
point_world = body_to_world(
    point_body,
    np.asarray([10.0, 20.0, 1.0]),
    [0.0, 0.0, half_sqrt_two, half_sqrt_two],
)
np.testing.assert_allclose(point_body, [1.2, 0.0, 0.0], atol=1e-8)
np.testing.assert_allclose(point_world, [10.0, 21.2, 1.0], atol=1e-8)
print("RAW_STEREO_GEOMETRY_TEST_PASSED depth=%.3f baseline=%.5f" %
      (result["depth_m"], result["baseline_m"]))
