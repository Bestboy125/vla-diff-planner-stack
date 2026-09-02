#!/usr/bin/env python3
"""Validate live Isaac/Pegasus, MAVROS and AirStack streams in ROS domain 43."""

import json
import time

import rclpy
from airstack_msgs.msg import Odometry as AirStackOdometry
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener


class RuntimeValidator(Node):
    def __init__(self) -> None:
        super().__init__("airstack_runtime_validator")
        self.samples = {}
        self.first_time = {}
        self.last_time = {}
        self.details = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image, "/robot_1/sensors/front_camera/image/rgb", self.image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, "/robot_1/sensors/front_camera/image/camera_info",
            self.camera_info_cb, 10,
        )
        self.create_subscription(
            PoseStamped, "/pegasus0/state/pose",
            lambda msg: self.record("pegasus_pose", msg.header.frame_id), 10,
        )
        self.create_subscription(
            Imu, "/pegasus0/sensors/imu",
            lambda msg: self.record("imu", msg.header.frame_id), qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/robot_1/odometry_conversion/odometry", self.odom_cb, 10,
        )
        self.create_subscription(
            State, "/robot_1/interface/mavros/state", self.state_cb, 10,
        )
        self.create_subscription(
            AirStackOdometry, "/robot_1/trajectory_controller/tracking_point",
            lambda _msg: self.record("tracking"), 10,
        )
        self.create_subscription(
            RollPitchYawrateThrust,
            "/robot_1/interface/cmd_roll_pitch_yawrate_thrust",
            lambda _msg: self.record("command"), 10,
        )

    def record(self, name: str, detail=None) -> None:
        now = time.monotonic()
        self.samples[name] = self.samples.get(name, 0) + 1
        self.first_time.setdefault(name, now)
        self.last_time[name] = now
        if detail is not None:
            self.details[name] = detail

    def image_cb(self, msg: Image) -> None:
        self.record("image", {"width": msg.width, "height": msg.height, "frame": msg.header.frame_id})

    def camera_info_cb(self, msg: CameraInfo) -> None:
        self.record(
            "camera_info",
            {"width": msg.width, "height": msg.height, "frame": msg.header.frame_id},
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.record(
            "odometry",
            {"frame": msg.header.frame_id, "child": msg.child_frame_id},
        )

    def state_cb(self, msg: State) -> None:
        self.record(
            "mavlink",
            {"connected": msg.connected, "armed": msg.armed, "mode": msg.mode},
        )

    def result(self):
        rates = {}
        for name, count in self.samples.items():
            span = self.last_time[name] - self.first_time[name]
            rates[name] = (count - 1) / span if count > 1 and span > 0 else None
        transforms = {}
        for parent, child in (("map", "base_link"), ("base_link", "camera_front")):
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                transforms[f"{parent}->{child}"] = {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                }
            except Exception as exc:
                transforms[f"{parent}->{child}"] = {"error": str(exc)}
        return {"samples": self.samples, "rates_hz": rates, "details": self.details, "tf": transforms}


def main() -> int:
    rclpy.init()
    node = RuntimeValidator()
    required = {"image", "camera_info", "pegasus_pose", "imu", "odometry", "mavlink", "tracking", "command"}
    deadline = time.monotonic() + 25.0
    minimum_runtime = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() >= minimum_runtime and required.issubset(node.samples):
            break
    result = node.result()
    passed = required.issubset(node.samples)
    image = result["details"].get("image", {})
    info = result["details"].get("camera_info", {})
    passed = passed and image.get("width") == info.get("width") and image.get("width", 0) >= 320
    passed = passed and image.get("height") == info.get("height") and image.get("height", 0) >= 240
    passed = passed and result["details"].get("mavlink", {}).get("connected") is True
    passed = passed and all("error" not in value for value in result["tf"].values())
    print(("RUNTIME_VALIDATION_PASS " if passed else "RUNTIME_VALIDATION_FAIL ") + json.dumps(result, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
