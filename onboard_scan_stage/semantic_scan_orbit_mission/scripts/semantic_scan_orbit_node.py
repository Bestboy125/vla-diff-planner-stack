#!/usr/bin/env python3
"""Orchestrate a serpentine scan, immediate chair stop, stereo orbit and resume."""
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Empty, String

sys.path.insert(0, os.path.dirname(__file__))
from scan_route_geometry import (ScanMissionError, generate_world_serpentine,
                                 horizontal_distance, validate_request)


def odom_position(message):
    point = message.pose.pose.position
    return (float(point.x), float(point.y), float(point.z))


def odom_yaw(message):
    q = message.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def linear_speed(message):
    velocity = message.twist.twist.linear
    return math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)


def distance3(first, second):
    return math.sqrt(sum((float(first[index]) - float(second[index])) ** 2 for index in range(3)))


class SemanticScanOrbitMission(object):
    NAVIGATION_PHASES = ("NAVIGATING", "RETURNING")
    ORBIT_TERMINAL_STATES = ("SUCCEEDED", "FAILED", "REJECTED", "CANCELLED")

    def __init__(self):
        gp = rospy.get_param
        self.execution_enabled = bool(gp("~execution_enabled", False))
        self.world_frame = str(gp("~world_frame", "world"))
        self.body_frame = str(gp("~body_frame", "base_link"))
        self.scan_width = float(gp("~scan_width_m", 6.0))
        self.scan_length = float(gp("~scan_length_m", 10.0))
        self.sweep_count = int(gp("~sweep_count", 5))
        self.waypoint_spacing = float(gp("~scan_waypoint_spacing_m", 1.0))
        self.turn_samples = int(gp("~turn_samples", 6))
        self.turn_bulge_ratio = float(gp("~turn_bulge_ratio", 0.45))
        self.max_goto_leg = float(gp("~max_world_goto_leg_m", 1.8))
        self.waypoint_timeout = float(gp("~waypoint_timeout", 60.0))
        self.target_label = str(gp("~target_label", "chair"))
        self.target_timeout = float(gp("~target_timeout", 0.50))
        self.stop_velocity_tolerance = float(gp("~stop_velocity_tolerance", 0.15))
        self.stop_settle_time = float(gp("~stop_settle_time", 0.60))
        self.orbit_status_timeout = float(gp("~orbit_status_timeout", 1800.0))
        self.post_orbit_cooldown = float(gp("~post_orbit_detection_cooldown", 5.0))
        self.processed_target_radius = float(gp("~processed_target_radius_m", 1.0))
        self.orbit_radius = float(gp("~orbit_radius_m", 1.5))
        self.orbit_laps = float(gp("~orbit_laps", 1.0))
        self.orbit_direction = str(gp("~orbit_direction", "clockwise"))
        self.orbit_yaw_mode = str(gp("~orbit_yaw_mode", "face_center"))
        self.odom_timeout = float(gp("~odom_timeout", 0.30))
        self.altitude_reference_z = float(gp("~altitude_reference_z_m", 0.0))
        self.min_altitude = float(gp("~min_operating_altitude", 0.30))
        self.max_altitude = float(gp("~max_operating_altitude", 2.0))

        self.lock = threading.RLock()
        self.latest_obstacle_cloud = None
        self.obstacle_resolution = float(gp('/drone_0_diff_planner_node/grid_map/resolution', 0.1))
        self.skipped_waypoints = []
        self.consecutive_skips = 0
        self.obstacle_check_at = 0.0
        rospy.Subscriber('/drone_0_diff_planner_node/grid_map/occupancy_inflate',
                         PointCloud2, self.on_obstacle_cloud, queue_size=1)
        self.active = None
        self.phase = None
        self.route = []
        self.route_index = 0
        self.rejoin = None
        self.route_origin = None
        self.nav_goal_finishes_waypoint = False
        self.latest_odom = None
        self.latest_odom_received = rospy.Time(0)
        self.latest_fcu = None
        self.class_ready = False
        self.observation_not_before = rospy.Time(0)
        self.ignore_detection_until = rospy.Time(0)
        self.pending_target = None
        self.processed_targets = []
        self.stop_stable_since = None
        self.active_orbit_task_id = None
        self.orbit_started = rospy.Time(0)
        self.orbit_count = 0
        self.completed_orbit_count = 0

        action_name = str(gp("~atomic_action_name", "/atomic_skill_executor/execute"))
        self.client = actionlib.SimpleActionClient(action_name, ExecuteAtomicSkillAction)
        self.target_class_pub = rospy.Publisher(
            "/semantic_raw_stereo_node/target_class_command", String, queue_size=1)
        self.semantic_orbit_request_pub = rospy.Publisher(
            "/vla_diff_bridge/semantic_orbit_request", String, queue_size=1)
        self.semantic_orbit_cancel_pub = rospy.Publisher(
            "/vla_diff_bridge/semantic_orbit_cancel", String, queue_size=1)
        self.hover_pub = rospy.Publisher("~hover_stop", Empty, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=20, latch=True)
        self.route_pub = rospy.Publisher("~route_json", String, queue_size=1, latch=True)

        rospy.Subscriber("~request", String, self.on_request, queue_size=1)
        rospy.Subscriber("~cancel", String, self.on_cancel, queue_size=1)
        rospy.Subscriber("~odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("~fcu_state", State, self.on_fcu, queue_size=1)
        rospy.Subscriber("/semantic_raw_stereo_node/target_class_status", String,
                         self.on_target_class_status, queue_size=1)
        # Require a stable detection before interrupting, including far coarse targets.
        rospy.Subscriber("/semantic_raw_stereo_node/stable_target_world", PointStamped,
                         self.on_raw_target, queue_size=1)
        rospy.Subscriber("/semantic_raw_stereo_node/stable_coarse_target_world", PointStamped,
                         self.on_raw_target, queue_size=1)
        rospy.Subscriber("/semantic_orbit_executor/status", String,
                         self.on_orbit_status, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.on_timer)
        self.publish_status("READY", "semantic scan-orbit mission initialized")

    def publish_status(self, state, detail, extra=None, request=None):
        active = request if request is not None else self.active
        payload = {
            "state": state,
            "detail": detail,
            "execution_enabled": self.execution_enabled,
            "task_id": active.get("task_id") if active else None,
            "route_index": self.route_index,
            "route_waypoint_count": len(self.route),
            "completed_waypoints": max(0, min(self.route_index, len(self.route))-len(getattr(self, 'skipped_waypoints', []))),
            "skipped_waypoints": list(getattr(self, 'skipped_waypoints', [])),
            "orbit_attempts": self.orbit_count,
            "completed_orbits": self.completed_orbit_count,
            "time_unix_ms": int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        if (extra and 'goal_world' in extra) or state in ('CHAIR_DETECTED', 'ORBIT_COMPLETED', 'REJOIN_ALIGN', 'SCAN_RESUMED', 'FAILED'):
            rospy.loginfo('[scan_checkpoint] %s', json.dumps(payload, separators=(',', ':')))

    def on_odom(self, message):
        with self.lock:
            self.latest_odom = message
            self.latest_odom_received = rospy.Time.now()

    def on_fcu(self, message):
        with self.lock:
            self.latest_fcu = (bool(message.connected), bool(message.armed), rospy.Time.now())

    def require_flight_ready(self):
        if self.latest_odom is None:
            raise ScanMissionError("odometry is unavailable")
        age = (rospy.Time.now() - self.latest_odom_received).to_sec()
        if age < -0.05 or age > self.odom_timeout:
            raise ScanMissionError("odometry is stale")
        if self.latest_odom.header.frame_id != self.world_frame:
            raise ScanMissionError("odometry frame does not match world frame")
        child = self.latest_odom.child_frame_id.strip()
        if child and child != self.body_frame:
            raise ScanMissionError("odometry child frame does not match body frame")
        if self.latest_fcu is None:
            raise ScanMissionError("FCU state is unavailable")
        connected, armed, stamp = self.latest_fcu
        if (rospy.Time.now() - stamp).to_sec() > 1.0 or not connected or not armed:
            raise ScanMissionError("vehicle must already be connected and armed")
        position = odom_position(self.latest_odom)
        altitude = position[2] - self.altitude_reference_z
        if not self.min_altitude <= altitude <= self.max_altitude:
            raise ScanMissionError("vehicle altitude %.3f m AGL is outside safety bounds" % altitude)
        return self.latest_odom

    def on_request(self, message):
        request = None
        try:
            request = validate_request(json.loads(message.data))
            with self.lock:
                if not self.execution_enabled:
                    raise ScanMissionError("execution is disabled by the onboard safety gate")
                if self.active is not None:
                    raise ScanMissionError("another scan-orbit mission is active")
                odom = self.require_flight_ready()
                if not self.client.wait_for_server(rospy.Duration(0.5)):
                    raise ScanMissionError("atomic skill action server is unavailable")
                if "path_tangent" not in rospy.get_param(
                        "/atomic_skill_executor/goto_yaw_modes", []):
                    raise ScanMissionError("atomic executor lacks path_tangent yaw; restart updated stack")
                if not rospy.get_param("/drone_0_traj_server/supports_yaw_settle", False):
                    raise ScanMissionError("trajectory server lacks yaw settling; restart updated stack")
                if self.target_class_pub.get_num_connections() < 1:
                    raise ScanMissionError("raw-stereo YOLO target-class subscriber is unavailable")
                if self.semantic_orbit_request_pub.get_num_connections() < 1:
                    raise ScanMissionError("verified stereo semantic-orbit executor is unavailable")
                self.route = generate_world_serpentine(
                    odom_position(odom),
                    width_m=self.scan_width, length_m=self.scan_length,
                    sweep_count=self.sweep_count,
                    waypoint_spacing_m=self.waypoint_spacing,
                    turn_samples=self.turn_samples,
                    turn_bulge_ratio=self.turn_bulge_ratio)
                if not self.route:
                    raise ScanMissionError("generated scan route is empty")
                self.active = request
                self.route_origin = odom_position(odom)
                self.rejoin = None
                self.skipped_waypoints = []
                self.consecutive_skips = 0
                self.phase = "WAITING_FOR_YOLO"
                self.route_index = 0
                self.nav_goal_finishes_waypoint = False
                self.class_ready = False
                self.observation_not_before = rospy.Time.now()
                self.ignore_detection_until = rospy.Time(0)
                self.pending_target = None
                self.processed_targets = []
                self.stop_stable_since = None
                self.active_orbit_task_id = None
                self.orbit_count = 0
                self.completed_orbit_count = 0
                self.publish_route(odom)
                self.target_class_pub.publish(String(data="chair"))
                self.publish_status(
                    "WAITING_FOR_YOLO",
                    "fixed scan generated; waiting for continuous chair detector readiness",
                    {"origin_world": list(odom_position(odom)),
                     "start_yaw_rad": odom_yaw(odom), "scan_heading_rad": 0.0,
                     "expansion_axis": "+Y", "scan_yaw_mode": "path_tangent"})
        except (ValueError, ScanMissionError) as exc:
            self.publish_status("REJECTED", str(exc), request=request)

    def publish_route(self, odom):
        payload = {
            "frame_id": self.world_frame,
            "origin_world": list(odom_position(odom)),
            "start_yaw_rad": odom_yaw(odom),
            "scan_heading_rad": 0.0,
            "expansion_axis": "+Y",
            "scan_yaw_mode": "path_tangent",
            "world_x_extent_m": self.scan_width,
            "world_y_extent_m": self.scan_length,
            "scan_width_m": self.scan_width,
            "scan_length_m": self.scan_length,
            "sweep_count": self.sweep_count,
            "waypoints": [
                {"index": index, "world": list(item["world"]),
                 "kind": item["kind"], "sweep_index": item["sweep_index"],
                 "yaw_rad": item["yaw_rad"]}
                for index, item in enumerate(self.route)
            ],
        }
        self.route_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def on_target_class_status(self, message):
        with self.lock:
            if self.active is None or message.data.strip().lower() != "chair":
                return
            self.class_ready = True
            if self.phase != "WAITING_FOR_YOLO":
                return
            self.observation_not_before = rospy.Time.now()
            self.publish_status("YOLO_READY", "continuous YOLO-World chair detection is active")
            try:
                self.dispatch_route_leg()
            except ScanMissionError as exc:
                self.fail(str(exc))

    def dispatch_route_leg(self):
        if self.active is None:
            return
        odom = self.require_flight_ready()
        if self.route_index >= len(self.route):
            request = self.active
            self.hover_pub.publish(Empty())
            self.publish_status(
                "MISSION_COMPLETED", "all preset scan waypoints completed",
                {"processed_chairs": len(self.processed_targets),
                 "skipped_waypoints": getattr(self, 'skipped_waypoints', []),
                 "coverage_complete": not getattr(self, 'skipped_waypoints', [])}, request=request)
            self.reset()
            return
        current = odom_position(odom)
        route_target = self.route[self.route_index]["world"]
        if self.goal_occupied(route_target):
            self.begin_obstacle_skip()
            return
        remaining = distance3(current, route_target)
        if remaining <= 0.15:
            reached = self.route_progress_extra()
            reached["completed_waypoints"] = self.route_index + 1 - len(getattr(self, 'skipped_waypoints', []))
            self.publish_status(
                "WAYPOINT_REACHED", "scan waypoint already within completion tolerance",
                reached)
            self.route_index += 1
            self.dispatch_route_leg()
            return
        goal_target = route_target
        self.nav_goal_finishes_waypoint = remaining <= self.max_goto_leg
        if not self.nav_goal_finishes_waypoint:
            ratio = self.max_goto_leg / remaining
            goal_target = tuple(current[index] + (route_target[index] - current[index]) * ratio
                                for index in range(3))
        goal = ExecuteAtomicSkillGoal()
        goal.skill = "GOTO_WORLD"
        goal.center.x, goal.center.y, goal.center.z = goal_target
        goal.center_frame = "world"
        goal.yaw_mode = "path_tangent"
        goal.timeout = self.waypoint_timeout
        self.phase = "NAVIGATING" if self.nav_goal_finishes_waypoint else "RETURNING"
        # Never send the next goal inside SimpleActionClient's done callback:
        # it sets DONE after that callback, corrupting the new goal's state.
        # Polling also avoids callback/mission-lock inversion during cancellation.
        self.navigation_started = time.monotonic()
        self.client.send_goal(goal)
        self.navigation_target = goal_target
        self.publish_status(
            self.phase,
            "Diff-Planner is executing a preset scan waypoint" if self.nav_goal_finishes_waypoint
            else "returning toward the interrupted scan waypoint in a bounded leg",
            dict(self.route_progress_extra(), goal_world=list(goal_target),
                 goal_yaw_rad=math.atan2(goal_target[1] - current[1],
                                         goal_target[0] - current[0]),
                 remaining_to_route_waypoint_m=remaining))

    def route_progress_extra(self):
        item = self.route[self.route_index] if self.route_index < len(self.route) else None
        return {
            "waypoint_index": self.route_index,
            "waypoint_number": self.route_index + 1,
            "waypoint_count": len(self.route),
            "sweep_index": item["sweep_index"] if item else self.sweep_count - 1,
            "waypoint_kind": item["kind"] if item else "complete",
        }

    def on_obstacle_cloud(self, message):
        # Cache only. Scan at most once per second, not on every cloud callback.
        self.latest_obstacle_cloud = (message, time.monotonic())

    def goal_occupied(self, target):
        cached = getattr(self, 'latest_obstacle_cloud', None)
        if cached is None or time.monotonic()-cached[1] > 1.0:
            return False  # Unknown is NOT evidence of occupancy or free space.
        cloud = cached[0]
        if cloud.header.frame_id != self.world_frame:
            return False
        half = self.obstacle_resolution * 0.51
        if not math.isfinite(half) or half <= 0:
            return False
        return any(all(abs(float(point[i])-target[i]) <= half for i in range(3))
                   for point in point_cloud2.read_points(cloud, field_names=('x','y','z'), skip_nans=True))

    def begin_obstacle_skip(self):
        if getattr(self, 'consecutive_skips', 0) >= 3:
            self.fail('three consecutive occupied waypoints skipped; stopping to avoid blind route advancement')
            return
        self.client.cancel_goal()
        self.hover_pub.publish(Empty())
        self.phase = 'SKIP_SETTLING'
        self.skip_started = time.monotonic()
        self.skip_stable_since = None
        self.publish_status('WAYPOINT_BLOCKED', 'inflated obstacle cloud confirms occupied target; cancelling and settling',
                            dict(self.route_progress_extra(), goal_world=self.route[self.route_index]['world']))

    def advance_obstacle_skip(self):
        try:
            odom = self.require_flight_ready()
            if time.monotonic()-self.skip_started > 10.0:
                raise ScanMissionError('could not settle before skipping occupied waypoint')
            if ((self.client.gh is not None and self.client.simple_state != 2)
                    or linear_speed(odom) > self.stop_velocity_tolerance):
                self.skip_stable_since = None
                return
            if self.skip_stable_since is None:
                self.skip_stable_since = time.monotonic()
                return
            if time.monotonic()-self.skip_stable_since < self.stop_settle_time:
                return
            self.skipped_waypoints.append(self.route_index)
            self.consecutive_skips += 1
            self.publish_status('WAYPOINT_SKIPPED', 'occupied waypoint skipped; next goal still requires Diff-Planner collision-free planning',
                                self.route_progress_extra())
            self.route_index += 1
            self.dispatch_route_leg()
        except ScanMissionError as exc:
            self.fail(str(exc))

    def capture_rejoin(self, odom):
        end = self.route[self.route_index]["world"]
        start = self.route[self.route_index - 1]["world"] if self.route_index else self.route_origin
        vector = tuple(end[i] - start[i] for i in range(3))
        length2 = sum(v*v for v in vector)
        if length2 < 1e-8:
            raise ScanMissionError("cannot rejoin a zero-length route segment")
        position = odom_position(odom)
        fraction = max(0.0, min(1.0, sum((position[i]-start[i])*vector[i] for i in range(3))/length2))
        self.rejoin = dict(index=self.route_index, interrupted_world=list(position),
                           anchor=tuple(start[i]+fraction*vector[i] for i in range(3)),
                           yaw=math.atan2(vector[1], vector[0]))

    def dispatch_rejoin(self):
        odom = self.require_flight_ready()
        if self.rejoin is None or self.route_index != self.rejoin['index']:
            raise ScanMissionError("scan rejoin checkpoint/index mismatch")
        current, anchor = odom_position(odom), self.rejoin['anchor']
        distance = distance3(current, anchor)
        if distance <= 0.15:
            self.phase = 'REJOIN_SETTLING'
            self.rejoin['stable_since'] = None
            self.navigation_started = time.monotonic()
            return
        scale = min(1.0, self.max_goto_leg / distance)
        target = tuple(current[i]+scale*(anchor[i]-current[i]) for i in range(3))
        goal = ExecuteAtomicSkillGoal()
        goal.skill = 'GOTO_WORLD'; goal.center_frame = 'world'
        goal.center.x, goal.center.y, goal.center.z = target
        goal.yaw_mode = 'path_tangent'; goal.timeout = self.waypoint_timeout
        self.phase = 'REJOINING'; self.navigation_started = time.monotonic()
        self.client.send_goal(goal)
        self.publish_status('REJOINING', 'returning to saved world scan segment; detection triggers paused',
                            dict(self.rejoin, goal_world=target, measured_world=current))

    def advance_rejoin(self):
        try:
            odom = self.require_flight_ready()
            if time.monotonic() - self.rejoin.get('last_status', 0.0) >= 1.0:
                self.rejoin['last_status'] = time.monotonic()
                self.publish_status('REJOIN_PROGRESS', 'return/alignment in progress; scan index frozen',
                                    dict(self.rejoin, phase=self.phase, measured_world=odom_position(odom),
                                         measured_yaw=odom_yaw(odom)))
            if time.monotonic() - self.rejoin['started'] > 180.0:
                raise ScanMissionError('scan rejoin total timeout')
            if time.monotonic() - self.navigation_started > self.waypoint_timeout + 2.0:
                raise ScanMissionError('scan rejoin stage timeout')
            if self.phase == 'REJOINING':
                if self.client.simple_state != 2:
                    return
                result = self.client.get_result()
                if self.client.get_state() != GoalStatus.SUCCEEDED or not result or not result.success:
                    raise ScanMissionError('return-to-route action failed')
                self.dispatch_rejoin()
                return
            if distance3(odom_position(odom), self.rejoin['anchor']) > 0.25:
                raise ScanMissionError('drifted away from saved rejoin point during settling/alignment')
            error = math.atan2(math.sin(self.rejoin['yaw']-odom_yaw(odom)),
                               math.cos(self.rejoin['yaw']-odom_yaw(odom)))
            if self.phase == 'REJOIN_ALIGN':
                if self.client.simple_state != 2:
                    return
                result = self.client.get_result()
                if self.client.get_state() != GoalStatus.SUCCEEDED or not result or not result.success:
                    raise ScanMissionError('scan heading alignment failed')
                if abs(error) > 0.10:
                    raise ScanMissionError('scan heading alignment not confirmed by odometry')
                self.phase = 'REJOIN_SETTLING'; self.rejoin['stable_since'] = None
            if (linear_speed(odom) > self.stop_velocity_tolerance
                    or abs(odom.twist.twist.angular.z) > 0.10):
                self.rejoin['stable_since'] = None
                return
            if self.rejoin.get('stable_since') is None:
                self.rejoin['stable_since'] = time.monotonic()
                return
            if time.monotonic() - self.rejoin['stable_since'] < self.stop_settle_time:
                return
            if abs(error) > 0.10:
                goal = ExecuteAtomicSkillGoal(); goal.skill = 'ROTATE'
                goal.direction = 'left' if error > 0 else 'right'
                goal.angle = abs(error); goal.timeout = self.waypoint_timeout
                self.phase = 'REJOIN_ALIGN'; self.navigation_started = time.monotonic()
                self.client.send_goal(goal)
                self.publish_status('REJOIN_ALIGN', 'aligning with original scan segment', dict(self.rejoin))
                return
            self.publish_status('SCAN_RESUMED', 'rejoin position, low speed and original heading confirmed',
                                dict(self.rejoin, measured_world=odom_position(odom), measured_yaw=odom_yaw(odom)))
            self.rejoin = None
            self.ignore_detection_until = rospy.Time.now() + rospy.Duration(self.post_orbit_cooldown)
            self.observation_not_before = self.ignore_detection_until
            self.dispatch_route_leg()
        except ScanMissionError as exc:
            self.fail(str(exc))

    def on_navigation_feedback(self, feedback):
        with self.lock:
            if self.active is None or self.phase not in self.NAVIGATION_PHASES:
                return
            self.publish_status(
                self.phase, "Diff-Planner waypoint progress",
                dict(self.route_progress_extra(), progress=float(feedback.progress),
                     planner_waypoint_index=int(feedback.waypoint_index),
                     planner_waypoint_count=int(feedback.waypoint_count)))

    def on_navigation_done(self, state, result):
        with self.lock:
            if self.active is None or self.phase not in self.NAVIGATION_PHASES:
                return
            if not (state == GoalStatus.SUCCEEDED and result and result.success):
                self.fail(result.message if result else "scan navigation returned no result")
                return
            if self.nav_goal_finishes_waypoint:
                self.consecutive_skips = 0
                reached = self.route_progress_extra()
                reached["completed_waypoints"] = self.route_index + 1 - len(getattr(self, 'skipped_waypoints', []))
                self.publish_status(
                    "WAYPOINT_REACHED", "Diff-Planner completed the preset scan waypoint",
                    reached)
                self.route_index += 1
            try:
                self.dispatch_route_leg()
            except ScanMissionError as exc:
                self.fail(str(exc))

    def target_already_processed(self, target):
        return any(horizontal_distance(target, previous) <= self.processed_target_radius
                   for previous in self.processed_targets)

    def on_raw_target(self, message):
        with self.lock:
            if (self.active is None or self.phase not in self.NAVIGATION_PHASES
                    or not self.class_ready or rospy.Time.now() < self.ignore_detection_until):
                return
            if message.header.frame_id != self.world_frame:
                return
            if message.header.stamp < self.observation_not_before:
                return
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if age < 0.0 or age > self.target_timeout:
                return
            target = (float(message.point.x), float(message.point.y), float(message.point.z))
            if not all(math.isfinite(value) for value in target) or self.target_already_processed(target):
                return
            self.pending_target = target
            try:
                self.capture_rejoin(self.require_flight_ready())
            except ScanMissionError as exc:
                self.fail(str(exc))
                return
            self.phase = "STOPPING_FOR_CHAIR"
            self.stop_stable_since = None
            self.client.cancel_goal()
            # Recoverable planner stop, not the latched emergency-stop path.
            self.hover_pub.publish(Empty())
            self.publish_status(
                "CHAIR_DETECTED", "stable stereo chair detected; scan navigation cancelled",
                dict(self.route_progress_extra(), target_world=list(target), rejoin_checkpoint=dict(self.rejoin)))

    def on_timer(self, _event):
        with self.lock:
            if self.active is None:
                return
            now = rospy.Time.now()
            if self.phase == 'SKIP_SETTLING':
                self.advance_obstacle_skip()
                return
            if self.phase in ('REJOINING', 'REJOIN_SETTLING', 'REJOIN_ALIGN'):
                self.advance_rejoin()
                return
            if self.phase in self.NAVIGATION_PHASES:
                try:
                    self.require_flight_ready()
                except ScanMissionError as exc:
                    self.fail(str(exc))
                    return
                if time.monotonic() - getattr(self, 'obstacle_check_at', 0.0) >= 1.0:
                    self.obstacle_check_at = time.monotonic()
                    if self.goal_occupied(self.route[self.route_index]['world']):
                        self.begin_obstacle_skip()
                        return
                # SimpleGoalState.DONE=2, not GoalStatus.SUCCEEDED=3. Only this
                # state proves actionlib has completed its transition callback.
                if self.client.simple_state == 2:
                    self.on_navigation_done(self.client.get_state(), self.client.get_result())
                elif time.monotonic() - self.navigation_started > self.waypoint_timeout + 2.0:
                    self.fail("scan waypoint execution timed out")
                else:
                    self.publish_status(self.phase, "waiting for waypoint completion",
                                        self.route_progress_extra())
                return
            if self.phase == "STOPPING_FOR_CHAIR":
                try:
                    odom = self.require_flight_ready()
                except ScanMissionError as exc:
                    self.fail(str(exc))
                    return
                if linear_speed(odom) > self.stop_velocity_tolerance:
                    self.stop_stable_since = None
                    return
                if self.stop_stable_since is None:
                    self.stop_stable_since = now
                    return
                if (now - self.stop_stable_since).to_sec() >= self.stop_settle_time:
                    try:
                        self.request_stereo_orbit()
                    except ScanMissionError as exc:
                        self.fail(str(exc))
                return
            if (self.phase == "ORBIT_REQUESTED"
                    and (now - self.orbit_started).to_sec() > self.orbit_status_timeout):
                self.fail("stereo semantic orbit status timed out")

    def request_stereo_orbit(self):
        if self.semantic_orbit_request_pub.get_num_connections() < 1:
            raise ScanMissionError("verified stereo semantic-orbit executor disconnected")
        self.orbit_count += 1
        self.active_orbit_task_id = "%s-orbit-%d" % (self.active["task_id"], self.orbit_count)
        request = {
            "task_id": self.active_orbit_task_id,
            "target_label": "chair",
            "target_world_hint": list(self.pending_target),
            "radius_m": self.orbit_radius,
            "laps": self.orbit_laps,
            "direction": self.orbit_direction,
            "yaw_mode": self.orbit_yaw_mode,
            "keep_current_altitude": True,
            "world_frame": self.world_frame,
            "body_frame": self.body_frame,
            "received_unix_ms": int(time.time() * 1000),
        }
        self.phase = "ORBIT_REQUESTED"
        self.orbit_started = rospy.Time.now()
        self.publish_status(
            "STOPPED_FOR_CHAIR", "odometry confirmed the vehicle is stationary",
            {"target_world": list(self.pending_target)})
        self.semantic_orbit_request_pub.publish(
            String(data=json.dumps(request, separators=(",", ":"))))
        self.publish_status(
            "ORBIT_REQUESTED",
            "requested verified D435 stereo orbit: clockwise, radius 1.5 m, one lap",
            {"orbit_task_id": self.active_orbit_task_id})

    def on_orbit_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            if (self.active is None or self.phase != "ORBIT_REQUESTED"
                    or status.get("task_id") != self.active_orbit_task_id):
                return
            state = str(status.get("state", ""))
            if state not in self.ORBIT_TERMINAL_STATES:
                self.publish_status(
                    "ORBITING", "verified stereo semantic orbit is in progress",
                    {"orbit_task_id": self.active_orbit_task_id,
                     "orbit_state": state, "orbit_detail": status.get("detail", "")})
                return
            if state != "SUCCEEDED":
                self.fail("stereo semantic orbit %s: %s" %
                          (state.lower(), status.get("detail", "no detail")))
                return
            self.completed_orbit_count += 1
            completed_target = status.get("target_world", self.pending_target)
            if (not isinstance(completed_target, (list, tuple)) or len(completed_target) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v)
                               for v in completed_target)):
                self.fail("orbit succeeded without a valid final target position")
                return
            self.processed_targets.append(tuple(completed_target))
            self.publish_status(
                "ORBIT_COMPLETED",
                "chair orbit completed; returning to saved scan segment before resuming",
                {"orbit_task_id": self.active_orbit_task_id,
                 "resume_waypoint_index": self.route_index})
            self.active_orbit_task_id = None
            self.pending_target = None
            self.ignore_detection_until = rospy.Time.now() + rospy.Duration(self.post_orbit_cooldown)
            self.observation_not_before = self.ignore_detection_until
            try:
                if self.rejoin is None:
                    raise ScanMissionError('missing scan interruption checkpoint')
                self.rejoin['started'] = time.monotonic()
                self.dispatch_rejoin()
            except ScanMissionError as exc:
                self.fail(str(exc))

    def on_cancel(self, _message):
        with self.lock:
            if self.active is None:
                return
            request = self.active
            self.client.cancel_goal()
            if self.active_orbit_task_id is not None:
                self.semantic_orbit_cancel_pub.publish(
                    String(data=json.dumps({"reason": "operator_hold",
                                            "task_id": self.active_orbit_task_id})))
            self.hover_pub.publish(Empty())
            self.publish_status("CANCELLED", "operator HOLD cancelled scan-orbit mission",
                                request=request)
            self.reset()

    def fail(self, detail):
        request = self.active
        self.client.cancel_goal()
        if self.active_orbit_task_id is not None:
            self.semantic_orbit_cancel_pub.publish(
                String(data=json.dumps({"reason": "scan_mission_failed",
                                        "task_id": self.active_orbit_task_id})))
        self.hover_pub.publish(Empty())
        self.publish_status("FAILED", detail, request=request)
        self.reset()

    def reset(self):
        self.rejoin = None
        self.active = None
        self.phase = None
        self.route = []
        self.route_index = 0
        self.nav_goal_finishes_waypoint = False
        self.class_ready = False
        self.pending_target = None
        self.stop_stable_since = None
        self.active_orbit_task_id = None


if __name__ == "__main__":
    rospy.init_node("semantic_scan_orbit_mission")
    SemanticScanOrbitMission()
    rospy.spin()
