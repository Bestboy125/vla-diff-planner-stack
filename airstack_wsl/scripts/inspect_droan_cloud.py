#!/usr/bin/env python3
"""Inspect live DROAN point clouds around the VLA pole benchmark corridor."""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class CloudInspector(Node):
    def __init__(self) -> None:
        super().__init__("droan_cloud_inspector")
        self.map_cloud = None
        self.camera_cloud = None
        self.disparity = None
        self.foreground = None
        self.background = None
        self.create_subscription(
            PointCloud2,
            "/robot_1/droan/fg_bg_cloud",
            lambda message: setattr(self, "map_cloud", message),
            10,
        )
        self.create_subscription(
            DisparityImage,
            "/robot_1/sensors/front_camera/disparity",
            lambda message: setattr(self, "disparity", message),
            10,
        )
        self.create_subscription(
            Image,
            "/robot_1/droan/foreground_expanded",
            lambda message: setattr(self, "foreground", message),
            10,
        )
        self.create_subscription(
            Image,
            "/robot_1/droan/background_expanded",
            lambda message: setattr(self, "background", message),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/robot_1/droan/expansion_cloud",
            lambda message: setattr(self, "camera_cloud", message),
            10,
        )


def xyz(message: PointCloud2) -> np.ndarray:
    values = point_cloud2.read_points_numpy(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    values = np.asarray(values, dtype=np.float64).reshape(-1, 3)
    return values[np.all(np.isfinite(values), axis=1)]


def image_array(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding in ("32fc1", "32fc"):
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        itemsize = 4
    elif encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        itemsize = 2
    elif encoding in ("8uc1", "mono8"):
        dtype = np.uint8
        itemsize = 1
    else:
        raise ValueError(f"unsupported diagnostic image encoding: {message.encoding}")
    return np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=message.data,
        strides=(message.step, itemsize),
    )


def image_summary(message: Image) -> dict:
    values = image_array(message)
    valid = np.isfinite(values) & (values > 0)
    # The analytically fused pole occupies this measured image region while
    # hovering at 1.2 m in the benchmark scene.
    pole = values[0:174, 62:87]
    pole_valid = np.isfinite(pole) & (pole > 0)
    return {
        "encoding": message.encoding,
        "shape": [int(message.height), int(message.width)],
        "valid": int(np.count_nonzero(valid)),
        "minimum": float(np.min(values[valid])) if np.any(valid) else None,
        "maximum": float(np.max(values[valid])) if np.any(valid) else None,
        "pole_region_valid": int(np.count_nonzero(pole_valid)),
        "pole_region_minimum": (
            float(np.min(pole[pole_valid])) if np.any(pole_valid) else None
        ),
        "pole_region_maximum": (
            float(np.max(pole[pole_valid])) if np.any(pole_valid) else None
        ),
    }


def summarize(node: CloudInspector) -> dict:
    map_points = xyz(node.map_cloud)
    camera_points = xyz(node.camera_cloud)

    direction = np.asarray([0.9124236720043529, 0.40924692151083203])
    along = map_points[:, :2] @ direction
    cross = np.abs(
        -map_points[:, 0] * direction[1] + map_points[:, 1] * direction[0]
    )
    corridor = (
        (along >= 2.0)
        & (along <= 5.0)
        & (cross <= 0.80)
        & (map_points[:, 2] >= 0.30)
        & (map_points[:, 2] <= 2.20)
    )

    expected_pole_xy = np.asarray([12.027 - 8.38, -13.463 - (-15.10)])
    pole_distance = np.linalg.norm(map_points[:, :2] - expected_pole_xy, axis=1)
    pole_band = (pole_distance <= 0.80) & (map_points[:, 2] >= 0.0)

    # Camera frame follows ROS optical convention: +z forward, +x right,
    # +y down. The target pole is roughly 3.6 m forward and 1.6 m left.
    camera_pole = (
        (camera_points[:, 2] >= 2.5)
        & (camera_points[:, 2] <= 4.5)
        & (camera_points[:, 0] >= -2.5)
        & (camera_points[:, 0] <= -0.8)
    )
    nearest_pole_index = int(np.argmin(pole_distance))

    def bounds(points: np.ndarray):
        if not len(points):
            return None
        return {
            "minimum": np.min(points, axis=0).tolist(),
            "maximum": np.max(points, axis=0).tolist(),
        }

    return {
        "disparity": {
            **image_summary(node.disparity.image),
            "f": float(node.disparity.f),
            "t": float(node.disparity.t),
            "min_disparity": float(node.disparity.min_disparity),
            "max_disparity": float(node.disparity.max_disparity),
        },
        "foreground_expanded": image_summary(node.foreground),
        "background_expanded": image_summary(node.background),
        "map": {
            "frame": node.map_cloud.header.frame_id,
            "points": int(len(map_points)),
            "bounds": bounds(map_points),
            "corridor_points": int(np.count_nonzero(corridor)),
            "minimum_corridor_cross_track_m": (
                float(np.min(cross[(along >= 2.0) & (along <= 5.0)]))
                if np.any((along >= 2.0) & (along <= 5.0))
                else None
            ),
            "expected_pole_xy": expected_pole_xy.tolist(),
            "points_near_expected_pole": int(np.count_nonzero(pole_band)),
            "minimum_pole_xy_distance_m": float(np.min(pole_distance)),
            "nearest_pole_point": map_points[nearest_pole_index].tolist(),
            "corridor_bounds": bounds(map_points[corridor]),
        },
        "camera": {
            "frame": node.camera_cloud.header.frame_id,
            "points": int(len(camera_points)),
            "bounds": bounds(camera_points),
            "points_in_expected_pole_frustum": int(np.count_nonzero(camera_pole)),
            "expected_pole_frustum_bounds": bounds(camera_points[camera_pole]),
        },
    }


def main() -> int:
    rclpy.init()
    node = CloudInspector()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not all(
        item is not None
        for item in (
            node.map_cloud,
            node.camera_cloud,
            node.disparity,
            node.foreground,
            node.background,
        )
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not all(
        item is not None
        for item in (
            node.map_cloud,
            node.camera_cloud,
            node.disparity,
            node.foreground,
            node.background,
        )
    ):
        raise TimeoutError("DROAN diagnostic streams were not all received")
    print("DROAN_CLOUD_INSPECTION " + json.dumps(summarize(node), sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
