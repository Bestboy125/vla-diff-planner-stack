#!/usr/bin/env python3
"""Record the exact ROS RGB stream consumed by OpenVLA to a video file."""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from capture_vla_observation import rgb_array


class RGBRecorder(Node):
    def __init__(self, output: Path, output_fps: float) -> None:
        super().__init__("openvla_rgb_video_recorder")
        self.output = output
        self.output_fps = output_fps
        self.writer = None
        self.last_received_at = None
        self.source_frames = 0
        self.encoded_frames = 0
        self.started_at = time.monotonic()
        self.create_subscription(
            Image,
            "/robot_1/sensors/front_camera/image/rgb",
            self.on_rgb,
            qos_profile_sensor_data,
        )

    def on_rgb(self, message: Image) -> None:
        now = time.monotonic()
        rgb = rgb_array(message)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.writer is None:
            height, width = bgr.shape[:2]
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.writer = cv2.VideoWriter(
                str(self.output),
                cv2.VideoWriter_fourcc(*"MJPG"),
                self.output_fps,
                (width, height),
            )
            if not self.writer.isOpened():
                raise RuntimeError(f"could not open video writer: {self.output}")

        elapsed = now - self.started_at
        label = f"OpenVLA RGB input | t={elapsed:05.1f}s | source={self.source_frames:04d}"
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 24), (0, 0, 0), -1)
        cv2.putText(
            bgr,
            label,
            (6, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        if self.last_received_at is None:
            repeats = 1
        else:
            repeats = max(1, min(int(round((now - self.last_received_at) * self.output_fps)), 20))
        for _ in range(repeats):
            self.writer.write(bgr)
        self.last_received_at = now
        self.source_frames += 1
        self.encoded_frames += repeats

    def close(self) -> dict:
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        return {
            "output": str(self.output),
            "source_frames": self.source_frames,
            "encoded_frames": self.encoded_frames,
            "output_fps": self.output_fps,
            "duration_s": self.encoded_frames / self.output_fps,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--output-fps", type=float, default=10.0)
    parser.add_argument("--max-duration", type=float, default=120.0)
    args = parser.parse_args()
    if not math.isfinite(args.output_fps) or args.output_fps <= 0.0:
        parser.error("--output-fps must be positive")

    rclpy.init()
    recorder = RGBRecorder(args.output, args.output_fps)
    deadline = time.monotonic() + args.max_duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if report["source_frames"] == 0:
        raise RuntimeError("no OpenVLA RGB frames were recorded")
    print("OPENVLA_RGB_RECORDING_PASS " + json.dumps(report, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
