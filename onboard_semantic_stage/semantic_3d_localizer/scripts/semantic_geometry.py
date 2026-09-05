#!/usr/bin/env python3
"""Dependency-free stereo and rigid-transform geometry."""
import math


def mat_vec(rotation, vector):
    return tuple(sum(rotation[i][j] * vector[j] for j in range(3)) for i in range(3))


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def stereo_point(bbox, disparity, fx, fy, cx, cy, baseline):
    if disparity <= 0 or fx <= 0 or fy <= 0 or baseline <= 0:
        raise ValueError("disparity, focal lengths and baseline must be positive")
    u = (bbox[0] + bbox[2]) * 0.5
    v = (bbox[1] + bbox[3]) * 0.5
    z = fx * baseline / disparity
    return pinhole_point(u, v, z, fx, fy, cx, cy)


def pinhole_point(u, v, depth, fx, fy, cx, cy):
    """Back-project one pixel and metric depth into a ROS optical frame."""
    if depth <= 0 or fx <= 0 or fy <= 0:
        raise ValueError("depth and focal lengths must be positive")
    return ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)


def quaternion_matrix(q):
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError("zero quaternion")
    s = 2.0 / n
    return (
        (1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)),
        (s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)),
        (s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)),
    )


def camera_to_world(point_camera, body_position, body_quaternion,
                    rotation_body_camera, translation_body_camera):
    point_body = camera_to_body(point_camera, rotation_body_camera, translation_body_camera)
    return body_to_world(point_body, body_position, body_quaternion)


def camera_to_body(point_camera, rotation_body_camera, translation_body_camera):
    """Transform a point from the calibrated optical frame into the body frame."""
    return add(mat_vec(rotation_body_camera, point_camera), translation_body_camera)


def body_to_world(point_body, body_position, body_quaternion):
    """Transform a body-frame point using the time-aligned odometry pose."""
    return add(mat_vec(quaternion_matrix(body_quaternion), point_body), body_position)


def standoff_goal(body_position, target_world, distance, keep_body_altitude=True):
    dx = target_world[0] - body_position[0]
    dy = target_world[1] - body_position[1]
    planar = math.hypot(dx, dy)
    if planar <= distance:
        raise ValueError("target is nearer than requested standoff")
    scale = (planar - distance) / planar
    goal_z = body_position[2] if keep_body_altitude else target_world[2]
    return (body_position[0] + dx*scale, body_position[1] + dy*scale, goal_z)
