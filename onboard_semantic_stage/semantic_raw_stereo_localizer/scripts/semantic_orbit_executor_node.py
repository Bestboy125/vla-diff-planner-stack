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
from semantic_orbit_contract import (SemanticOrbitError, altitude_agl,
                                     build_staged_orbit_spec,
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
        self.max_approach_stages = int(gp("~max_approach_stages", 20))
        self.target_match_radius = float(gp("~target_match_radius_m", 1.0))
        self.locked_target = None
        self.orbit_target = None
        self.approach_timeout = float(gp("~approach_timeout", 60.0))
        self.altitude_reference_z = float(gp("~altitude_reference_z_m", 0.0))
        self.min_operating_altitude = float(gp("~min_operating_altitude", 0.30))
        self.max_operating_altitude = float(gp("~max_operating_altitude", 2.0))
        self.skill_timeout = float(gp("~skill_timeout", 120.0))
        action_name = str(gp("~atomic_action_name", "/atomic_skill_executor/execute"))

        self.lock = threading.RLock()
        self.active = None
        self.phase = None
        self.active_started = rospy.Time(0)
        self.detection_started = rospy.Time(0)
        self.observation_not_before = rospy.Time(0)
        self.approach_stage_count = 0
        self.class_ready = False
        self.latest_odom = None
        self.latest_odom_received = rospy.Time(0)
        self.latest_fcu = None
        self.client = actionlib.SimpleActionClient(action_name, ExecuteAtomicSkillAction)
        self.target_class_pub = rospy.Publisher(
            "/semantic_raw_stereo_node/target_class_command", String, queue_size=1
        )
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        # Observation-only trace topic.  It is deliberately not /goal; the
        # atomic skill server remains the only publisher of executable points.
        self.entry_world_pub = rospy.Publisher(
            "~circle_entry_world", PointStamped, queue_size=1, latch=True
        )
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
        rospy.Subscriber("/semantic_raw_stereo_node/stable_coarse_target_world",
                         PointStamped, self.on_coarse_target, queue_size=1)
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
        altitude_agl(position[2], self.altitude_reference_z,
                     self.min_operating_altitude, self.max_operating_altitude)
        return position

    def on_request(self, message):
        request = None
        try:
            request = validate_semantic_orbit_request(json.loads(message.data))
            with self.lock:
                if not self.execution_enabled:
                    raise SemanticOrbitError("execution is disabled by the onboard safety gate")
                if self.active is not None:
                    raise SemanticOrbitError("another semantic orbit task is active")
                self.require_flight_ready()
                self.active = request
                self.locked_target = request.get("target_world_hint")
                self.orbit_target = None
                self.phase = "WAITING_FOR_CLASS"
                self.active_started = rospy.Time.now()
                self.detection_started = self.active_started
                self.observation_not_before = self.active_started
                self.approach_stage_count = 0
                self.class_ready = False
                self.target_class_pub.publish(String(data=request["target_label"]))
                self.publish_status("WAITING_FOR_CLASS", "requested YOLO-World class update", request)
        except (ValueError, SemanticOrbitError) as exc:
            self.publish_status("REJECTED", str(exc), request)

    def on_target_class_status(self, message):
        with self.lock:
            if (self.active is None or self.phase != "WAITING_FOR_CLASS"
                    or message.data.strip().lower() != self.active["target_label"]):
                return
            self.class_ready = True
            self.phase = "DETECTING"
            self.detection_started = rospy.Time.now()
            self.observation_not_before = self.detection_started
            self.publish_status("DETECTING", "target class active; waiting for stable 3-D target", self.active)

    def on_coarse_target(self, message):
        self.on_stable_target(message, coarse=True)

    def on_stable_target(self, message, coarse=False):
        with self.lock:
            request = self.active
            if request is None or not self.class_ready or self.phase != "DETECTING":
                return
            if message.header.frame_id != self.world_frame:
                self.finish_rejected("stable target frame does not match onboard world frame")
                return
            if message.header.stamp < self.observation_not_before:
                return
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if age < 0.0 or age > self.target_timeout:
                return
            try:
                current = self.require_flight_ready()
                target = (message.point.x, message.point.y, message.point.z)
                if not all(math.isfinite(v) for v in target):
                    raise SemanticOrbitError("non-finite semantic target")
                if (self.locked_target is not None and
                        math.dist(target, self.locked_target) > self.target_match_radius):
                    self.publish_status("TARGET_MISMATCH",
                                        "candidate outside target lock; waiting, no motion",
                                        request, {"target_world": list(target),
                                                  "locked_target_world": list(self.locked_target)})
                    return
                self.locked_target = target
                spec = build_staged_orbit_spec(
                    request, target, current, self.max_approach_leg
                )
                if not self.client.wait_for_server(rospy.Duration(0.25)):
                    raise SemanticOrbitError("atomic skill action server is unavailable")
                entry_message = PointStamped()
                entry_message.header.stamp = message.header.stamp
                entry_message.header.frame_id = self.world_frame
                (entry_message.point.x, entry_message.point.y,
                 entry_message.point.z) = spec["entry_world"]
                self.entry_world_pub.publish(entry_message)
                if coarse and not spec["approach_required"]:
                    self.publish_status("WAITING_FOR_PRECISE_TARGET",
                                        "coarse estimate cannot authorize circle entry or orbit",
                                        request)
                    return
                if spec["approach_required"]:
                    self.start_approach(request, target, spec)
                    return
                self.start_orbit(request, target, spec)
            except SemanticOrbitError as exc:
                self.finish_rejected(str(exc))

    def start_approach(self, request, target, spec):
        if self.approach_stage_count >= self.max_approach_stages:
            raise SemanticOrbitError(
                "circle entry is still too far after %d approach stages"
                % self.max_approach_stages
            )
        waypoint = spec["approach_waypoint_world"]
        goal = ExecuteAtomicSkillGoal()
        goal.skill = "GOTO_WORLD"
        goal.center.x, goal.center.y, goal.center.z = waypoint
        goal.center_frame = "world"
        goal.timeout = self.approach_timeout
        stage_number = self.approach_stage_count + 1
        self.class_ready = False
        self.phase = "APPROACHING"
        self.client.send_goal(
            goal,
            done_cb=lambda state, result: self.on_approach_done(
                request, stage_number, state, result
            ),
            feedback_cb=lambda feedback: self.on_skill_feedback(request, feedback),
        )
        self.publish_status(
            "APPROACHING",
            "moving toward the circle in a safety-limited stage",
            request,
            {"stage": stage_number, "max_stages": self.max_approach_stages,
             "target_world": list(target),
             "circle_entry_world": list(spec["entry_world"]),
             "approach_waypoint_world": list(waypoint),
             "remaining_to_entry_m": spec["approach_leg_m"]},
        )

    def start_orbit(self, request, target, spec):
        self.orbit_target = target
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
             "circle_entry_world": list(spec["entry_world"]),
             "approach_leg_m": spec["approach_leg_m"]},
        )

    def on_approach_done(self, request, stage_number, state, result):
        with self.lock:
            if self.active is None or self.active.get("task_id") != request.get("task_id"):
                return
            success = bool(result and result.success and state == GoalStatus.SUCCEEDED)
            if not success:
                detail = result.message if result else "approach skill returned no result"
                self.publish_status("FAILED", detail, request, {"stage": stage_number})
                self.active = None
                self.phase = None
                self.class_ready = False
                self.approach_stage_count = 0
                return
            self.approach_stage_count = stage_number
            self.phase = "WAITING_FOR_CLASS"
            self.class_ready = False
            self.detection_started = rospy.Time.now()
            self.observation_not_before = self.detection_started
            self.target_class_pub.publish(String(data=request["target_label"]))
            self.publish_status(
                "REDETECTING",
                "approach stage complete; refreshing the target before continuing",
                request,
                {"completed_stages": self.approach_stage_count,
                 "max_stages": self.max_approach_stages},
            )

    def on_skill_feedback(self, request, feedback):
        state = "APPROACHING" if self.phase == "APPROACHING" else str(
            feedback.status or "ORBITING"
        )
        detail = (
            "atomic approach progress"
            if self.phase == "APPROACHING"
            else "atomic orbit progress"
        )
        self.publish_status(
            state,
            detail,
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
            self.publish_status("SUCCEEDED" if success else "FAILED", detail, request,
                                {"target_world": list(self.orbit_target)}
                                if success and self.orbit_target is not None else None)
            self.active = None
            self.phase = None
            self.class_ready = False
            self.approach_stage_count = 0

    def finish_rejected(self, detail):
        request = self.active
        self.publish_status("REJECTED", detail, request)
        self.active = None
        self.phase = None
        self.class_ready = False
        self.approach_stage_count = 0

    def on_cancel(self, _message):
        with self.lock:
            if self.active is None:
                return
            request = self.active
            self.client.cancel_goal()
            self.active = None
            self.phase = None
            self.class_ready = False
            self.approach_stage_count = 0
            self.publish_status("CANCELLED", "operator HOLD cancelled semantic orbit", request)

    def on_timer(self, _event):
        with self.lock:
            if self.active is None or self.phase not in ("WAITING_FOR_CLASS", "DETECTING"):
                return
            if (rospy.Time.now() - self.detection_started).to_sec() > self.detection_timeout:
                self.finish_rejected("stable semantic target was not found before timeout")


if __name__ == "__main__":
    rospy.init_node("semantic_orbit_executor")
    SemanticOrbitExecutor()
    rospy.spin()
