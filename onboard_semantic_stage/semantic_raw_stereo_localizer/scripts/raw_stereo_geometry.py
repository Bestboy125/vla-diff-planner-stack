#!/usr/bin/env python3
"""Dependency-light raw rectified stereo geometry and rigid transforms."""
import math

import cv2
import numpy as np


def select_bbox_feature_cluster(points, bbox, min_points=4):
    """Keep one spatially coherent feature component, biased toward box centre."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(points) <= min_points:
        return np.arange(len(points), dtype=np.int32)
    x1, y1, x2, y2 = [float(value) for value in bbox]
    width, height = x2 - x1, y2 - y1
    radius = max(18.0, 0.18 * min(width, height))
    unvisited = set(range(len(points)))
    components = []
    while unvisited:
        seed = unvisited.pop()
        component = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            candidates = list(unvisited)
            if not candidates:
                break
            distances = np.linalg.norm(points[candidates] - points[current], axis=1)
            neighbors = [candidates[index] for index in np.where(distances <= radius)[0]]
            for neighbor in neighbors:
                unvisited.remove(neighbor)
                component.append(neighbor)
                queue.append(neighbor)
        components.append(component)
    center = np.asarray([(x1 + x2) * 0.5, (y1 + y2) * 0.5])
    diagonal = max(float(np.hypot(width, height)), 1.0)

    def score(component):
        cluster_center = np.mean(points[component], axis=0)
        center_prior = 1.0 - min(1.0, float(np.linalg.norm(cluster_center - center)) / diagonal)
        return len(component) * (0.55 + 0.45 * center_prior)

    best = max(components, key=score)
    if len(best) < min_points:
        raise ValueError("no coherent target feature cluster has enough matches")
    return np.asarray(sorted(best), dtype=np.int32)


def validate_projection_matrices(left_projection, right_projection):
    left = np.asarray(left_projection, dtype=np.float64).reshape(3, 4)
    right = np.asarray(right_projection, dtype=np.float64).reshape(3, 4)
    if not np.isfinite(left).all() or not np.isfinite(right).all():
        raise ValueError("projection matrices must be finite")
    if left[0, 0] <= 0.0 or left[1, 1] <= 0.0 or right[0, 0] <= 0.0:
        raise ValueError("projection matrices must contain positive focal lengths")
    left_center_x = -left[0, 3] / left[0, 0]
    right_center_x = -right[0, 3] / right[0, 0]
    baseline = right_center_x - left_center_x
    if not 0.02 <= abs(baseline) <= 0.20:
        raise ValueError("stereo baseline %.6fm is outside the expected range" % baseline)
    return left, right, baseline


def triangulate_rectified(points_left, points_right, left_projection, right_projection,
                          min_disparity_px=1.0, max_epipolar_error_px=1.5,
                          min_depth=0.35, max_depth=6.0, mad_scale=3.5,
                          max_depth_mad=0.50, prefer_near_cluster=False,
                          min_cluster_points=4, depth_cluster_gap=0.25):
    left, right, baseline = validate_projection_matrices(left_projection, right_projection)
    points_left = np.asarray(points_left, dtype=np.float64).reshape(-1, 2)
    points_right = np.asarray(points_right, dtype=np.float64).reshape(-1, 2)
    if len(points_left) != len(points_right) or not len(points_left):
        raise ValueError("left/right matches must be nonempty and equal length")
    if not np.isfinite(points_left).all() or not np.isfinite(points_right).all():
        raise ValueError("matched pixels must be finite")

    expected_sign = 1.0 if baseline > 0.0 else -1.0
    disparity = expected_sign * (points_left[:, 0] - points_right[:, 0])
    epipolar_error = np.abs(points_left[:, 1] - points_right[:, 1])
    keep = (disparity >= min_disparity_px) & (epipolar_error <= max_epipolar_error_px)
    points_left, points_right = points_left[keep], points_right[keep]
    epipolar_error = epipolar_error[keep]
    if len(points_left) < 2:
        raise ValueError("insufficient positive-disparity epipolar matches")

    homogeneous = cv2.triangulatePoints(left, right, points_left.T, points_right.T)
    valid_w = np.abs(homogeneous[3]) > 1e-9
    xyz = np.full((homogeneous.shape[1], 3), np.nan, dtype=np.float64)
    xyz[valid_w] = (homogeneous[:3, valid_w] / homogeneous[3, valid_w]).T
    keep = np.isfinite(xyz).all(axis=1)
    keep &= (xyz[:, 2] >= min_depth) & (xyz[:, 2] <= max_depth)
    xyz, points_left, epipolar_error = xyz[keep], points_left[keep], epipolar_error[keep]
    if len(xyz) < 2:
        raise ValueError("no triangulated points in the configured depth range")

    depth_cluster_size = len(xyz)
    if prefer_near_cluster:
        order = np.argsort(xyz[:, 2])
        sorted_depth = xyz[order, 2]
        split_after = np.where(
            np.diff(sorted_depth) > np.maximum(
                float(depth_cluster_gap), 0.06 * sorted_depth[:-1]))[0]
        clusters = np.split(order, split_after + 1)
        clusters = [cluster for cluster in clusters if len(cluster) >= int(min_cluster_points)]
        if not clusters:
            raise ValueError("no coherent near-depth cluster has enough stereo points")
        selected = clusters[0]
        xyz, points_left = xyz[selected], points_left[selected]
        epipolar_error = epipolar_error[selected]
        depth_cluster_size = len(selected)

    depth_median = float(np.median(xyz[:, 2]))
    depth_mad = float(np.median(np.abs(xyz[:, 2] - depth_median)))
    if depth_mad > max_depth_mad:
        raise ValueError("stereo depth MAD %.3fm exceeds %.3fm" % (depth_mad, max_depth_mad))
    if depth_mad > 1e-6:
        sigma = 1.4826 * depth_mad
        keep = np.abs(xyz[:, 2] - depth_median) <= mad_scale * sigma
        xyz, points_left, epipolar_error = xyz[keep], points_left[keep], epipolar_error[keep]
        depth_median = float(np.median(xyz[:, 2]))

    return {
        "points_left_camera": xyz,
        "points_left_px": points_left,
        "depth_m": depth_median,
        "depth_mad_m": depth_mad,
        "baseline_m": abs(float(baseline)),
        "depth_cluster_size": int(depth_cluster_size),
        "median_epipolar_error_px": float(np.median(epipolar_error)),
    }


def bbox_center_point(bbox, depth, left_projection):
    projection = np.asarray(left_projection, dtype=np.float64).reshape(3, 4)
    fx, fy, cx, cy = projection[0, 0], projection[1, 1], projection[0, 2], projection[1, 2]
    u = 0.5 * (float(bbox[0]) + float(bbox[2]))
    v = 0.5 * (float(bbox[1]) + float(bbox[3]))
    return np.asarray([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64)


def quaternion_matrix(quaternion):
    x, y, z, w = [float(value) for value in quaternion]
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    scale = 2.0 / norm
    return np.asarray([
        [1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y)],
    ])


def camera_to_body(point_camera, body_t_camera):
    transform = np.asarray(body_t_camera, dtype=np.float64).reshape(4, 4)
    return transform[:3, :3].dot(np.asarray(point_camera)) + transform[:3, 3]


def body_to_world(point_body, body_position, body_quaternion):
    return quaternion_matrix(body_quaternion).dot(np.asarray(point_body)) + np.asarray(body_position)


def standoff_goal(body_position, target_world, distance, keep_body_altitude=True):
    body = np.asarray(body_position, dtype=np.float64)
    target = np.asarray(target_world, dtype=np.float64)
    delta = target[:2] - body[:2]
    planar = float(np.linalg.norm(delta))
    if planar <= distance:
        raise ValueError("target is nearer than requested standoff")
    goal = body.copy()
    goal[:2] += delta * ((planar - distance) / planar)
    goal[2] = body[2] if keep_body_altitude else target[2]
    return goal
