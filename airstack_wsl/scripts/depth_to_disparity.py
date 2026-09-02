#!/usr/bin/env python3
"""Convert Isaac metric depth to the stereo disparity contract used by DROAN."""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage


class DepthToDisparity(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("isaac_depth_to_disparity")
        self.baseline_m = args.baseline_m
        self.sim_max_range_m = args.sim_max_range_m
        self.fx = None
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(DisparityImage, args.disparity_topic, qos)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera_info, qos)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos)
        self.frames = 0
        self.get_logger().info(
            f"DEPTH_TO_DISPARITY_READY depth={args.depth_topic} "
            f"disparity={args.disparity_topic} baseline={self.baseline_m:.3f}m"
        )

    def on_camera_info(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0:
            self.fx = float(message.k[0])

    def on_depth(self, message: Image) -> None:
        if self.fx is None:
            return
        if message.encoding not in ("32FC1", "32FC"):
            self.get_logger().error(f"Unsupported Isaac depth encoding: {message.encoding}")
            return
        depth = np.ndarray(
            shape=(message.height, message.width),
            dtype=np.dtype("<f4" if not message.is_bigendian else ">f4"),
            buffer=message.data,
            strides=(message.step, 4),
        )
        working_depth = np.array(depth, dtype=np.float32, copy=True)
        if self.sim_max_range_m > 0.0:
            # Isaac uses +inf for rays that hit no geometry. In this bounded
            # simulation only, model those rays as a finite sensor max range;
            # otherwise DROAN correctly classifies the whole open sky as
            # unknown and refuses every candidate. Never enable this for a
            # real sensor where unknown space must remain unknown.
            working_depth[~np.isfinite(working_depth)] = self.sim_max_range_m
        disparity = np.zeros(depth.shape, dtype=np.float32)
        valid = np.isfinite(working_depth) & (working_depth > 0.05)
        disparity[valid] = (self.fx * self.baseline_m) / working_depth[valid]

        output = DisparityImage()
        output.header = message.header
        output.image.header = message.header
        output.image.height = message.height
        output.image.width = message.width
        output.image.encoding = "32FC1"
        output.image.is_bigendian = False
        output.image.step = message.width * 4
        output.image.data = disparity.tobytes()
        output.f = float(self.fx)
        output.t = float(self.baseline_m)
        output.min_disparity = 0.0
        output.max_disparity = float(np.max(disparity)) if valid.any() else 0.0
        output.delta_d = 0.0
        output.valid_window.x_offset = 0
        output.valid_window.y_offset = 0
        output.valid_window.width = message.width
        output.valid_window.height = message.height
        self.publisher.publish(output)

        self.frames += 1
        if self.frames == 1:
            valid_depth = working_depth[valid]
            self.get_logger().info(
                "FIRST_DISPARITY_FRAME "
                f"shape={message.width}x{message.height} "
                f"depth_min={float(valid_depth.min()):.3f}m "
                f"depth_median={float(np.median(valid_depth)):.3f}m "
                f"disp_max={output.max_disparity:.3f}px"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--disparity-topic", required=True)
    parser.add_argument("--baseline-m", type=float, default=0.12)
    parser.add_argument(
        "--sim-max-range-m",
        type=float,
        default=0.0,
        help="simulation-only replacement depth for +inf pixels; zero disables",
    )
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    node = DepthToDisparity(parse_args())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
