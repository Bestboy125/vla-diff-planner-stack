#!/usr/bin/env python3
"""Capture one synchronized-enough RGB/proprio/depth observation for VLA tests."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ObservationCapture(Node):
    def __init__(self) -> None:
        super().__init__("vla_observation_capture")
        self.rgb = None
        self.depth = None
        self.odom = None
        self.create_subscription(
            Image,
            "/robot_1/sensors/front_camera/image/rgb",
            lambda message: setattr(self, "rgb", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/robot_1/sensors/front_camera/image/depth",
            lambda message: setattr(self, "depth", message),
            10,
        )
        self.create_subscription(
            Odometry,
            "/robot_1/odometry_conversion/odometry",
            lambda message: setattr(self, "odom", message),
            10,
        )


def rgb_array(message: Image) -> np.ndarray:
    channels = 4 if message.encoding.lower() in ("rgba8", "bgra8") else 3
    array = np.ndarray(
        (message.height, message.width, channels),
        dtype=np.uint8,
        buffer=message.data,
        strides=(message.step, channels, 1),
    )
    encoding = message.encoding.lower()
    if encoding == "bgr8":
        array = array[:, :, ::-1]
    elif encoding == "bgra8":
        array = array[:, :, [2, 1, 0]]
    elif encoding == "rgba8":
        array = array[:, :, :3]
    elif encoding != "rgb8":
        raise ValueError(f"unsupported RGB encoding: {message.encoding}")
    return np.ascontiguousarray(array)


def depth_array(message: Image) -> np.ndarray:
    if message.encoding not in ("32FC1", "32FC"):
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    return np.ndarray(
        (message.height, message.width),
        dtype=np.dtype("<f4" if not message.is_bigendian else ">f4"),
        buffer=message.data,
        strides=(message.step, 4),
    )


def depth_statistics(message: Image) -> dict:
    array = depth_array(message)
    valid = array[np.isfinite(array) & (array > 0.05)]
    h, w = array.shape
    center = array[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
    center = center[np.isfinite(center) & (center > 0.05)]
    if not valid.size:
        return {
            "minimum_m": None,
            "median_m": None,
            "center_minimum_m": None,
            "valid_fraction": 0.0,
        }
    return {
        "minimum_m": float(valid.min()),
        "median_m": float(np.median(valid)),
        "center_minimum_m": float(center.min()) if center.size else None,
        "valid_fraction": float(valid.size / array.size),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument(
        "--depth-image",
        type=Path,
        help="optional inverse-depth grayscale preview (near=white, invalid=black)",
    )
    parser.add_argument(
        "--depth-npy",
        type=Path,
        help="optional lossless float32 depth array for geometric diagnostics",
    )
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = ObservationCapture()
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not (node.rgb and node.depth and node.odom):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not (node.rgb and node.depth and node.odom):
        raise TimeoutError("RGB, depth and odometry were not all received")

    args.image.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    PILImage.fromarray(rgb_array(node.rgb)).save(args.image, quality=95)
    if args.depth_image is not None:
        args.depth_image.parent.mkdir(parents=True, exist_ok=True)
        depth = depth_array(node.depth)
        valid = np.isfinite(depth) & (depth > 0.05)
        preview = np.zeros(depth.shape, dtype=np.uint8)
        preview[valid] = np.asarray(
            255.0 * (1.0 - np.clip(depth[valid], 0.0, 25.0) / 25.0),
            dtype=np.uint8,
        )
        PILImage.fromarray(preview, mode="L").save(args.depth_image)
    if args.depth_npy is not None:
        args.depth_npy.parent.mkdir(parents=True, exist_ok=True)
        np.save(args.depth_npy, np.ascontiguousarray(depth_array(node.depth)))
    pose = node.odom.pose.pose
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    report = {
        "image": str(args.image),
        "rgb": {
            "width": node.rgb.width,
            "height": node.rgb.height,
            "encoding": node.rgb.encoding,
            "frame_id": node.rgb.header.frame_id,
        },
        "proprio": [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            math.degrees(yaw),
        ],
        "depth": depth_statistics(node.depth),
    }
    args.metadata.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("VLA_OBSERVATION_PASS " + json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
