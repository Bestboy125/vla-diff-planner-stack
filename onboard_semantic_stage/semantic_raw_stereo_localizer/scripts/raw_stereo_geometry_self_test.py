#!/usr/bin/env python3
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from raw_stereo_geometry import (bbox_center_point, select_bbox_feature_cluster,
                                 triangulate_rectified)


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
print("RAW_STEREO_GEOMETRY_TEST_PASSED depth=%.3f baseline=%.5f" %
      (result["depth_m"], result["baseline_m"]))
