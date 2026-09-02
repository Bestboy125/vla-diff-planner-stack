#!/usr/bin/env python3
"""Safely validate the AirStack -> MAVROS -> PX4 -> Pegasus flight loop."""

import json
import math
import os
import sys
import time

import rclpy
from airstack_msgs.msg import FixedTrajectory, Odometry as AirStackOdometry
from airstack_msgs.srv import RobotCommand, TakeoffLandingCommand, TrajectoryMode
from diagnostic_msgs.msg import KeyValue
from mav_msgs.msg import RollPitchYawrateThrust
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, String


NS = "/robot_1"


class FlightValidator(Node):
    def __init__(self) -> None:
        super().__init__("airstack_flight_validator")
        self.state = None
        self.odom = None
        self.tracking = None
        self.command_count = 0
        self.last_command = None
        self.tracking_count = 0
        self.takeoff_state = ""
        self.landing_state = ""
        self.metrics = {}

        self.create_subscription(State, f"{NS}/interface/mavros/state", self._state_cb, 10)
        self.create_subscription(
            Odometry, f"{NS}/odometry_conversion/odometry", self._odom_cb, 10
        )
        self.create_subscription(
            AirStackOdometry,
            f"{NS}/trajectory_controller/tracking_point",
            self._tracking_cb,
            10,
        )
        self.create_subscription(
            RollPitchYawrateThrust,
            f"{NS}/interface/cmd_roll_pitch_yawrate_thrust",
            self._command_cb,
            10,
        )

        self.create_subscription(
            String,
            f"{NS}/takeoff_landing_planner/takeoff_state",
            lambda msg: setattr(self, "takeoff_state", msg.data),
            10,
        )
        self.create_subscription(
            String,
            f"{NS}/takeoff_landing_planner/landing_state",
            lambda msg: setattr(self, "landing_state", msg.data),
            10,
        )

        self.fixed_pub = self.create_publisher(
            FixedTrajectory,
            f"{NS}/fixed_trajectory_generator/fixed_trajectory_command",
            1,
        )
        self.reset_integrators_pub = self.create_publisher(
            Empty, f"{NS}/control/reset_integrators", 1
        )
        self.robot_client = self.create_client(
            RobotCommand, f"{NS}/interface/robot_command"
        )
        self.trajectory_mode_client = self.create_client(
            TrajectoryMode, f"{NS}/trajectory_controller/set_trajectory_mode"
        )
        self.takeoff_landing_client = self.create_client(
            TakeoffLandingCommand,
            f"{NS}/takeoff_landing_planner/set_takeoff_landing_command",
        )
        self.set_mode_client = self.create_client(SetMode, f"{NS}/interface/mavros/set_mode")

    def _state_cb(self, msg) -> None:
        self.state = msg

    def _odom_cb(self, msg) -> None:
        self.odom = msg
        z = msg.pose.pose.position.z
        self.metrics["max_z"] = max(self.metrics.get("max_z", z), z)

    def _tracking_cb(self, msg) -> None:
        self.tracking = msg
        self.tracking_count += 1

    def _command_cb(self, msg) -> None:
        self.last_command = msg
        self.command_count += 1

    def wait_for(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(f"timeout waiting for {label}")

    def call(self, client, request, label: str, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service unavailable: {label}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done() or future.exception() is not None:
            raise RuntimeError(f"service failed: {label}: {future.exception()}")
        return future.result()

    def set_trajectory_mode(self, mode: int) -> None:
        request = TrajectoryMode.Request()
        request.mode = mode
        response = self.call(self.trajectory_mode_client, request, f"trajectory mode {mode}")
        if not response.success:
            raise RuntimeError(f"trajectory mode {mode} was rejected")

    def robot_command(self, command: int) -> None:
        request = RobotCommand.Request()
        request.command = command
        response = self.call(self.robot_client, request, f"robot command {command}")
        if not response.success:
            raise RuntimeError(f"robot command {command} was rejected")

    def takeoff_landing(self, command: int) -> None:
        request = TakeoffLandingCommand.Request()
        request.command = command
        response = self.call(
            self.takeoff_landing_client, request, f"takeoff/landing command {command}"
        )
        if not response.accepted:
            raise RuntimeError(f"takeoff/landing command {command} was rejected")

    def px4_mode(self, mode: str) -> bool:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        response = self.call(self.set_mode_client, request, f"PX4 mode {mode}")
        return bool(response.mode_sent)

    def publish_line(self, height: float = 0.5) -> None:
        message = FixedTrajectory()
        message.type = "Line"
        values = {
            "frame_id": "map",
            "length": "1.5",
            "height": f"{height:.3f}",
            "velocity": "0.35",
            "max_acceleration": "0.30",
        }
        message.attributes = [KeyValue(key=key, value=value) for key, value in values.items()]
        for _ in range(3):
            self.fixed_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.2)

    def reset_integrators(self) -> None:
        for _ in range(3):
            self.reset_integrators_pub.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.15)

    def landed(self) -> bool:
        return self.odom is not None and self.odom.pose.pose.position.z < 0.14

    def safe_recover(self) -> None:
        if self.state is None or not self.state.armed:
            return
        if self.landed():
            try:
                self.px4_mode("AUTO.LAND")
                self.wait_for(lambda: self.state is not None and not self.state.armed, 12.0, "ground auto-disarm")
            except Exception as exc:
                self.get_logger().error(f"ground AUTO.LAND failed: {exc}")
            if self.state is None or not self.state.armed:
                return
        try:
            self.takeoff_landing(TakeoffLandingCommand.Request.LAND)
            self.wait_for(self.landed, 20.0, "AirStack landing")
        except Exception as exc:
            self.get_logger().error(f"AirStack recovery landing failed: {exc}")

        if self.state is not None and self.state.armed and not self.landed():
            try:
                self.px4_mode("AUTO.LAND")
                self.wait_for(
                    lambda: self.landed() or (self.state is not None and not self.state.armed),
                    30.0,
                    "PX4 AUTO.LAND",
                )
            except Exception as exc:
                self.get_logger().error(f"PX4 AUTO.LAND recovery failed: {exc}")

        if self.state is not None and self.state.armed and self.landed():
            try:
                self.robot_command(RobotCommand.Request.DISARM)
                self.wait_for(lambda: self.state is not None and not self.state.armed, 8.0, "disarm")
            except Exception as exc:
                self.get_logger().error(f"ground disarm failed: {exc}")

    def run_validation(self) -> None:
        self.wait_for(
            lambda: self.state is not None and self.state.connected and self.odom is not None,
            20.0,
            "MAVLink and odometry",
        )
        if self.state.armed:
            raise RuntimeError("refusing to start: vehicle is already armed")

        self.wait_for(
            lambda: self.tracking_count >= 10 and self.command_count >= 10,
            10.0,
            "20 Hz AirStack control stream",
        )
        start = self.odom.pose.pose.position
        self.metrics.update(
            start_x=start.x,
            start_y=start.y,
            start_z=start.z,
            max_z=start.z,
            command_count_before_arm=self.command_count,
        )

        self.set_trajectory_mode(TrajectoryMode.Request.ROBOT_POSE)
        self.reset_integrators()
        baseline_commands = self.command_count
        self.wait_for(
            lambda: self.command_count >= baseline_commands + 30
            and self.last_command is not None
            and 0.35 <= self.last_command.thrust.z <= 0.75,
            5.0,
            "pre-OFFBOARD setpoint stream and hover thrust",
        )
        self.metrics["hover_thrust"] = self.last_command.thrust.z
        print(f"PHASE_READY hover_thrust={self.last_command.thrust.z:.3f}", flush=True)
        self.robot_command(RobotCommand.Request.REQUEST_CONTROL)
        self.wait_for(lambda: self.state is not None and self.state.mode == "OFFBOARD", 8.0, "OFFBOARD")
        print("PHASE_OFFBOARD", flush=True)

        self.robot_command(RobotCommand.Request.ARM)
        self.wait_for(lambda: self.state is not None and self.state.armed, 8.0, "arming")
        self.metrics["armed"] = True
        self.reset_integrators()
        print("PHASE_ARMED", flush=True)

        self.set_trajectory_mode(TrajectoryMode.Request.TRACK)
        self.takeoff_landing(TakeoffLandingCommand.Request.TAKEOFF)
        self.wait_for(
            lambda: self.odom is not None and self.odom.pose.pose.position.z >= 0.35,
            30.0,
            "AirStack takeoff altitude",
        )
        self.metrics["takeoff_z"] = self.odom.pose.pose.position.z
        print(f"PHASE_TAKEOFF z={self.metrics['takeoff_z']:.3f}", flush=True)

        hold_seconds = float(os.environ.get("AIRSTACK_TAKEOFF_HOLD_SEC", "0"))
        if hold_seconds > 0:
            hold_deadline = time.monotonic() + hold_seconds
            self.wait_for(
                lambda: time.monotonic() >= hold_deadline,
                hold_seconds + 2.0,
                "configured takeoff hold",
            )

        if os.environ.get("AIRSTACK_SKIP_LINE", "0").lower() not in ("1", "true", "yes"):
            pre_line_x = self.odom.pose.pose.position.x
            pre_line_y = self.odom.pose.pose.position.y
            self.publish_line(height=0.5)
            self.wait_for(
                lambda: self.odom is not None
                and math.hypot(
                    self.odom.pose.pose.position.x - pre_line_x,
                    self.odom.pose.pose.position.y - pre_line_y,
                )
                >= 0.35,
                35.0,
                "AirStack fixed trajectory displacement",
            )
            current = self.odom.pose.pose.position
            self.metrics["line_displacement"] = math.hypot(
                current.x - pre_line_x, current.y - pre_line_y
            )
            self.metrics["line_x"] = current.x
            self.metrics["line_y"] = current.y
            print(f"PHASE_LINE displacement={self.metrics['line_displacement']:.3f}", flush=True)
        else:
            self.metrics["line_skipped"] = True
            print("PHASE_HOVER_ONLY", flush=True)

        self.takeoff_landing(TakeoffLandingCommand.Request.LAND)
        self.wait_for(self.landed, 35.0, "AirStack landing altitude")
        self.px4_mode("AUTO.LAND")
        try:
            self.wait_for(lambda: self.state is not None and not self.state.armed, 12.0, "PX4 landed auto-disarm")
        except TimeoutError:
            self.robot_command(RobotCommand.Request.DISARM)
            self.wait_for(lambda: self.state is not None and not self.state.armed, 8.0, "disarm")
        self.px4_mode("AUTO.LOITER")
        self.wait_for(
            lambda: self.state is not None and self.state.mode == "AUTO.LOITER",
            8.0,
            "post-flight AUTO.LOITER",
        )
        self.set_trajectory_mode(TrajectoryMode.Request.ROBOT_POSE)
        self.reset_integrators()
        print("PHASE_LANDED", flush=True)

        self.metrics.update(
            final_z=self.odom.pose.pose.position.z,
            final_mode=self.state.mode,
            armed_final=self.state.armed,
            command_count_final=self.command_count,
            tracking_count_final=self.tracking_count,
        )


def main() -> int:
    rclpy.init()
    validator = FlightValidator()
    try:
        validator.run_validation()
        print("FLIGHT_VALIDATION_PASS " + json.dumps(validator.metrics, sort_keys=True))
        return 0
    except Exception as exc:
        print(f"FLIGHT_VALIDATION_FAIL {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        validator.safe_recover()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
