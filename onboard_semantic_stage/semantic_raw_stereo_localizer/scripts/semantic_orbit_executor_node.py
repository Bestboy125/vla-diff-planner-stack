#!/usr/bin/env python3
"""Board-side state machine chaining semantic localization to the atomic ORBIT skill."""
import json
import math
import os
import sys
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from atomic_skill_executor.msg import ExecuteAtomicSkillAction, ExecuteAtomicSkillGoal
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(__file__))
from semantic_orbit_contract import (SemanticOrbitError, build_world_orbit_spec,
                                     validate_semantic_orbit_request)


class SemanticOrbitExecutor(object):
    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.world_frame = str(gp("~world_frame", "world"))
        self.odom_timeout = float(gp("~odom_timeout", 0.30))
        self.target_timeout = float(gp("~target_timeout", 0.50))
        self.detection_timeout = float(gp("~detection_timeout", 15.0))
        self.max_approach_leg = float(gp("~max_approach_leg", 2.0))
        self.min_operating_altitude = float(gp("~min_operating_altitude", 0.30))
        self.max_operating_altitude = float(gp("~max_operating_altitude", 2.0))
        self.skill_timeout = float(gp("~skill_timeout", 120.0))
        action_name = str(gp("~atomic_action_name", "/atomic_skill_executor/execute"))

        self.lock = threading.RLock()
        self.active = None
        self.phase = None
        self.active_started = rospy.Time(0)
        self.class_ready = False
        self.latest_odom = None
        self.latest_odom_received = rospy.Time(0)
        self.latest_fcu = None
        self.client = actionlib.SimpleActionClient(action_name, ExecuteAtomicSkillAction)
        self.target_class_pub = rospy.Publisher(
            "/semantic_raw_stereo_node/target_class_command", String, queue_size=1
        )
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        rospy.Subscriber("~request", String, self.on_request, queue_size=1)
        rospy.Subscriber("~cancel", String, self.on_cancel, queue_size=1)
        rospy.Subscriber(
            "/semantic_raw_stereo_node/target_class_status",
            String,
            self.on_target_class_status,
            queue_size=1,
        )
        rospy.Subscriber(
            "/semantic_raw_stereo_node/stable_target_world",
            PointStamped,
            self.on_stable_target,
            queue_size=1,
        )
        rospy.Subscriber(str(gp("~odom_topic", "/ekf/ekf_odom")), Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(str(gp("~fcu_state_topic", "/mavros/state")), State, self.on_fcu, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.on_timer)
        self.publish_status("READY", "semantic orbit executor initialized")
        rospy.loginfo("semantic orbit executor ready; execution_enabled=%s", self.execution_enabled)

    def publish_status(self, state, detail, request=None, extra=None):
        payload = {
            "state": state,
            "detail": detail,
            "execution_enabled": self.execution_enabled,
            "task_id": request.get("task_id") if request else None,
            "target_label": request.get("target_label") if request else None,
            "time_unix_ms": int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def on_odom(self, message):
        with self.lock:
            self.latest_odom = message
            self.latest_odom_received = rospy.Time.now()

    def on_fcu(self, message):
        with self.lock:
            self.latest_fcu = (bool(message.connected), bool(message.armed), rospy.Time.now())

    def require_flight_ready(self):
        if self.latest_odom is None:
            raise SemanticOrbitError("odometry is unavailable")
        odom_age = (rospy.Time.now() - self.latest_odom_received).to_sec()
        if odom_age < 0.0 or odom_age > self.odom_timeout:
            raise SemanticOrbitError("odometry is stale")
        if self.latest_fcu is None:
            raise SemanticOrbitError("FCU state is unavailable")
        connected, armed, stamp = self.latest_fcu
        if (rospy.Time.now() - stamp).to_sec() > 1.0 or not connected or not armed:
            raise SemanticOrbitError("vehicle must already be connected and armed")
        point = self.latest_odom.pose.pose.position
        position = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in position):
            raise SemanticOrbitError("odometry position is non-finite")
        if not self.min_operating_altitude <= position[2] <= self.max_operating_altitude:
            raise SemanticOrbitError("vehicle altitude is outside the semantic-orbit safety band")
        return position

    def on_request(self, message):
        try:
            request = validate_semantic_orbit_request(json.loads(message.data))
            with self.lock:
                if not self.execution_enabled:
                    raise SemanticOrbitError("execution is disabled by the onboard safety gate")
                if self.active is not None:
                    raise SemanticOrbitError("another semantic orbit task is active")
                self.require_flight_ready()
                self.active = request
                self.phase = "WAITING_FOR_CLASS"
                self.active_started = rospy.Time.now()
                self.class_ready = False
                self.target_class_pub.publish(String(data=request["target_label"]))
                self.publish_status("WAITING_FOR_CLASS", "requested YOLO-World class update", request)
        except (ValueError, SemanticOrbitError) as exc:
            self.publish_status("REJECTED", str(exc))

    def on_target_class_status(self, message):
        with self.lock:
            if self.active is None or message.data.strip().lower() != self.active["target_label"]:
                return
            self.class_ready = True
            self.phase = "DETECTING"
            self.publish_status("DETECTING", "target class active; waiting for stable 3-D target", self.active)

    def on_stable_target(self, message):
        with self.lock:
            request = self.active
            if request is None or not self.class_ready:
                return
            if message.header.frame_id != self.world_frame:
                self.finish_rejected("stable target frame does not match onboard world frame")
                return
            if message.header.stamp < self.active_started:
                return
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if age < 0.0 or age > self.target_timeout:
                return
            try:
                current = self.require_flight_ready()
                target = (message.point.x, message.point.y, message.point.z)
                spec = build_world_orbit_spec(request, target, current, self.max_approach_leg)
                if not self.client.wait_for_server(rospy.Duration(0.25)):
                    raise SemanticOrbitError("atomic skill action server is unavailable")
                goal = ExecuteAtomicSkillGoal()
                goal.skill = "ORBIT"
                goal.center.x, goal.center.y, goal.center.z = spec["center"]
                goal.center_frame = "world"
                goal.radius = spec["radius_m"]
                goal.orbit_angle = spec["orbit_angle_rad"]
                goal.direction = spec["direction"]
                goal.yaw_mode = spec["yaw_mode"]
                goal.timeout = self.skill_timeout
                self.class_ready = False
                self.phase = "ORBITING"
                self.client.send_goal(
                    goal,
                    done_cb=lambda state, result: self.on_skill_done(request, state, result),
                    feedback_cb=lambda feedback: self.on_skill_feedback(request, feedback),
                )
                self.publish_status(
                    "ORBITING",
                    "atomic ORBIT accepted; its first waypoint is the 1.5 m circle entry",
                    request,
                    {"target_world": list(target), "orbit_center": list(spec["center"]),
                     "approach_leg_m": spec["approach_leg_m"]},
                )
            except SemanticOrbitError as exc:
                self.finish_rejected(str(exc))

    def on_skill_feedback(self, request, feedback):
        self.publish_status(
            str(feedback.status or "ORBITING"),
            "atomic orbit progress",
            request,
            {"progress": float(feedback.progress), "waypoint_index": int(feedback.waypoint_index),
             "waypoint_count": int(feedback.waypoint_count)},
        )

    def on_skill_done(self, request, state, result):
        with self.lock:
            if self.active is None or self.active.get("task_id") != request.get("task_id"):
                return
            success = bool(result and result.success and state == GoalStatus.SUCCEEDED)
            detail = result.message if result else "atomic skill returned no result"
            self.publish_status("SUCCEEDED" if success else "FAILED", detail, request)
            self.active = None
            self.phase = None
            self.class_ready = False

    def finish_rejected(self, detail):
        request = self.active
        self.publish_status("REJECTED", detail, request)
        self.active = None
        self.phase = None
        self.class_ready = False

    def on_cancel(self, _message):
        with self.lock:
            if self.active is None:
                return
            request = self.active
            self.client.cancel_goal()
            self.active = None
            self.phase = None
            self.class_ready = False
            self.publish_status("CANCELLED", "operator HOLD cancelled semantic orbit", request)

    def on_timer(self, _event):
        with self.lock:
            if self.active is None or self.phase not in ("WAITING_FOR_CLASS", "DETECTING"):
                return
            if (rospy.Time.now() - self.active_started).to_sec() > self.detection_timeout:
                self.finish_rejected("stable semantic target was not found before timeout")


if __name__ == "__main__":
    rospy.init_node("semantic_orbit_executor")
    SemanticOrbitExecutor()
    rospy.spin()
