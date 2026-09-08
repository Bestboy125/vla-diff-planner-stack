#!/usr/bin/env python3
"""D435-left far estimate -> staged approach -> stereo refinement and orbit."""
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
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String

# roslaunch enters through catkin's devel/lib wrapper rather than this scripts
# directory.  Add the real source directory before importing the pure helper.
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from hybrid_orbit_geometry import HybridOrbitError, next_handoff_leg, validate_request


TERMINAL_FAILURES = {"FAILED", "REJECTED", "CANCELLED"}


class HybridSemanticOrbitMission(object):
    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.world_frame = str(gp("~world_frame", "world"))
        self.body_frame = str(gp("~body_frame", "base_link"))
        self.altitude_reference_z = float(gp("~altitude_reference_z_m", 0.0))
        self.min_altitude = float(gp("~min_operating_altitude", 0.30))
        self.max_altitude = float(gp("~max_operating_altitude", 2.0))
        self.odom_timeout = float(gp("~odom_timeout", 0.30))
        self.handoff_distance = float(gp("~d435_handoff_distance_m", 4.0))
        self.max_target_range = float(gp("~max_coarse_target_range_m", 50.0))
        self.max_leg = float(gp("~max_approach_leg_m", 2.0))
        self.max_stages = int(gp("~max_approach_stages", 30))
        self.approach_timeout = float(gp("~approach_timeout", 60.0))

        self.lock = threading.RLock()
        self.active = None
        self.phase = None
        self.mono_child_id = None
        self.stereo_child_id = None
        self.coarse_target = None
        self.approach_stage = 0
        self.latest_odom = None
        self.latest_odom_received = rospy.Time(0)
        self.latest_fcu = None

        self.client = actionlib.SimpleActionClient(
            str(gp("~atomic_action_name", "/atomic_skill_executor/execute")),
            ExecuteAtomicSkillAction,
        )
        self.mono_request_pub = rospy.Publisher(
            "/vla_diff_bridge/monocular_semantic_orbit_request", String, queue_size=1
        )
        self.mono_cancel_pub = rospy.Publisher(
            "/vla_diff_bridge/monocular_semantic_orbit_cancel", String, queue_size=1
        )
        self.stereo_request_pub = rospy.Publisher(
            "/vla_diff_bridge/semantic_orbit_request", String, queue_size=1
        )
        self.stereo_cancel_pub = rospy.Publisher(
            "/vla_diff_bridge/semantic_orbit_cancel", String, queue_size=1
        )
        self.stereo_inference_pub = rospy.Publisher(
            "/semantic_raw_stereo_node/inference_enabled", Bool, queue_size=1, latch=True
        )
        self.hover_pub = rospy.Publisher("~hover_stop", Empty, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)

        rospy.Subscriber("~request", String, self.on_request, queue_size=1)
        rospy.Subscriber("~cancel", String, self.on_cancel, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("~fcu_state", State, self.on_fcu, queue_size=1)
        rospy.Subscriber(
            "/monocular_semantic_orbit/status", String, self.on_mono_status, queue_size=10
        )
        rospy.Subscriber(
            "/semantic_orbit_executor/status", String, self.on_stereo_status, queue_size=10
        )
        self.stereo_inference_pub.publish(Bool(data=True))
        self.publish_status("READY", "hybrid far-to-near semantic orbit initialized")

    def publish_status(self, state, detail, extra=None, request_override=None):
        request = request_override if request_override is not None else self.active
        payload = {
            "state": state,
            "detail": detail,
            "execution_enabled": self.execution_enabled,
            "task_id": request.get("task_id") if request else None,
            "target_label": request.get("target_label") if request else None,
            "approach_stage": self.approach_stage,
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

    def current_position(self):
        if self.latest_odom is None:
            raise HybridOrbitError("odometry is unavailable")
        if (rospy.Time.now() - self.latest_odom_received).to_sec() > self.odom_timeout:
            raise HybridOrbitError("odometry is stale")
        if self.latest_odom.header.frame_id != self.world_frame:
            raise HybridOrbitError("odometry frame does not match world frame")
        if self.latest_fcu is None:
            raise HybridOrbitError("FCU state is unavailable")
        connected, armed, stamp = self.latest_fcu
        if (rospy.Time.now() - stamp).to_sec() > 1.0 or not connected or not armed:
            raise HybridOrbitError("vehicle must already be connected and armed")
        point = self.latest_odom.pose.pose.position
        position = (float(point.x), float(point.y), float(point.z))
        agl = position[2] - self.altitude_reference_z
        if not self.min_altitude <= agl <= self.max_altitude:
            raise HybridOrbitError("vehicle altitude is outside hybrid mission safety bounds")
        return position

    def on_request(self, message):
        request = None
        try:
            request = validate_request(json.loads(message.data))
            with self.lock:
                if not self.execution_enabled:
                    raise HybridOrbitError("execution is disabled by the onboard safety gate")
                if self.active is not None:
                    raise HybridOrbitError("another hybrid semantic task is active")
                self.current_position()
                if not self.client.wait_for_server(rospy.Duration(0.5)):
                    raise HybridOrbitError("atomic skill action server is unavailable")
                if self.mono_request_pub.get_num_connections() < 1:
                    raise HybridOrbitError("D435-left localization executor is unavailable")
                if self.stereo_request_pub.get_num_connections() < 1:
                    raise HybridOrbitError("D435 semantic orbit executor is unavailable")
                self.active = request
                self.phase = "MONOCULAR_LOCALIZING"
                self.mono_child_id = request["task_id"] + ":mono"
                self.stereo_child_id = request["task_id"] + ":stereo"
                self.coarse_target = None
                self.approach_stage = 0
                child = dict(request)
                child["task_id"] = self.mono_child_id
                child["localize_only"] = True
                # The parent owns this gate so D435 stays paused not only during
                # A/B capture, but also throughout the coarse approach legs.
                child["manage_stereo_inference"] = False
                self.stereo_inference_pub.publish(Bool(data=False))
                self.mono_request_pub.publish(
                    String(data=json.dumps(child, separators=(",", ":")))
                )
                self.publish_status(
                    "MONOCULAR_LOCALIZING",
                    "capturing two D435 left views with measured 0.5-1.0 m baseline",
                    {"child_task_id": self.mono_child_id},
                )
        except (ValueError, HybridOrbitError) as exc:
            self.publish_status("REJECTED", str(exc), request_override=request)

    def on_mono_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            if self.active is None or payload.get("task_id") != self.mono_child_id:
                return
            state = str(payload.get("state", ""))
            if state in TERMINAL_FAILURES:
                self.finish("FAILED", "D435-left localization failed: %s" % payload.get("detail", state))
                return
            if state != "LOCALIZED":
                self.publish_status("MONOCULAR_" + state, payload.get("detail", state))
                return
            target = payload.get("target_world_m")
            if not isinstance(target, list) or len(target) != 3:
                self.finish("FAILED", "D435-left localizer returned no world target")
                return
            self.coarse_target = tuple(float(value) for value in target)
            current = self.current_position()
            distance = math.dist(current, self.coarse_target)
            if not math.isfinite(distance) or distance > self.max_target_range:
                self.finish("FAILED", "coarse target exceeds maximum hybrid range")
                return
            self.publish_status(
                "COARSE_TARGET_ESTIMATED",
                "D435-left target passed geometry gates",
                {"coarse_target_world": list(self.coarse_target), "coarse_range_m": distance},
            )
            self.start_next_approach()

    def start_next_approach(self):
        current = self.current_position()
        waypoint, distance = next_handoff_leg(
            current, self.coarse_target, self.handoff_distance, self.max_leg
        )
        if waypoint is None:
            self.start_d435_handoff(distance)
            return
        if self.approach_stage >= self.max_stages:
            self.finish("FAILED", "D435 handoff remains too far after maximum approach stages")
            return
        self.approach_stage += 1
        goal = ExecuteAtomicSkillGoal()
        goal.skill = "GOTO_WORLD"
        goal.center.x, goal.center.y, goal.center.z = waypoint
        goal.center_frame = self.world_frame
        goal.timeout = self.approach_timeout
        self.phase = "COARSE_APPROACHING"
        self.client.send_goal(goal, done_cb=self.on_approach_done,
                              feedback_cb=self.on_action_feedback)
        self.publish_status(
            "COARSE_APPROACHING", "Diff-Planner is executing a bounded coarse approach leg",
            {"coarse_target_world": list(self.coarse_target),
             "approach_waypoint_world": list(waypoint), "coarse_range_m": distance,
             "max_leg_m": self.max_leg},
        )

    def on_action_feedback(self, feedback):
        with self.lock:
            if self.active is None or self.phase != "COARSE_APPROACHING":
                return
            self.publish_status(
                "COARSE_APPROACHING", "Diff-Planner coarse approach progress",
                {"progress": float(feedback.progress),
                 "waypoint_index": int(feedback.waypoint_index),
                 "waypoint_count": int(feedback.waypoint_count)},
            )

    def on_approach_done(self, state, result):
        with self.lock:
            if self.active is None or self.phase != "COARSE_APPROACHING":
                return
            if not (state == GoalStatus.SUCCEEDED and result and result.success):
                self.finish("FAILED", result.message if result else "coarse approach returned no result")
                return
            try:
                self.start_next_approach()
            except HybridOrbitError as exc:
                self.finish("FAILED", str(exc))

    def start_d435_handoff(self, coarse_range):
        request = self.active
        self.phase = "D435_LOCALIZING"
        self.stereo_inference_pub.publish(Bool(data=True))
        child = {
            "task_id": self.stereo_child_id,
            "target_label": request["target_label"],
            "radius_m": 1.5,
            "laps": 1.0,
            "direction": "clockwise",
            "yaw_mode": "face_center",
            "keep_current_altitude": True,
        }
        self.stereo_request_pub.publish(String(data=json.dumps(child, separators=(",", ":"))))
        self.publish_status(
            "D435_HANDOFF",
            "coarse approach entered the D435 quality envelope; requesting fresh stereo target",
            {"coarse_range_m": coarse_range, "handoff_distance_m": self.handoff_distance,
             "child_task_id": self.stereo_child_id},
        )

    def on_stereo_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            if self.active is None or payload.get("task_id") != self.stereo_child_id:
                return
            state = str(payload.get("state", ""))
            if state == "SUCCEEDED":
                self.finish("SUCCEEDED", "D435-refined clockwise 1.5 m orbit completed")
            elif state in TERMINAL_FAILURES:
                self.finish("FAILED", "D435 refinement/orbit failed: %s" % payload.get("detail", state))
            elif state == "ORBITING":
                self.phase = "ORBITING"
                self.publish_status("ORBITING", payload.get("detail", "D435 refined orbit active"))
            else:
                self.publish_status("D435_" + state, payload.get("detail", state))

    def on_cancel(self, _message):
        with self.lock:
            if self.active is None:
                return
            self.client.cancel_goal()
            self.mono_cancel_pub.publish(String(data=json.dumps({"reason": "parent_cancel"})))
            self.stereo_cancel_pub.publish(String(data=json.dumps({"reason": "parent_cancel"})))
            self.hover_pub.publish(Empty())
            self.publish_status("CANCELLED", "operator HOLD cancelled hybrid semantic orbit")
            self.reset()

    def finish(self, state, detail):
        if state != "SUCCEEDED":
            self.client.cancel_goal()
            self.hover_pub.publish(Empty())
        self.publish_status(state, detail)
        self.reset()

    def reset(self):
        self.stereo_inference_pub.publish(Bool(data=True))
        self.active = None
        self.phase = None
        self.mono_child_id = None
        self.stereo_child_id = None
        self.coarse_target = None
        self.approach_stage = 0


if __name__ == "__main__":
    rospy.init_node("hybrid_semantic_orbit_mission")
    HybridSemanticOrbitMission()
    rospy.spin()
