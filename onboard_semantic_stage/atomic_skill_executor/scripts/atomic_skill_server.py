#!/usr/bin/env python3
"""Safety-gated atomic skills used by the staged semantic-orbit workflow."""
import math

import actionlib
import rospy
from atomic_skill_executor.msg import (
    ExecuteAtomicSkillAction,
    ExecuteAtomicSkillFeedback,
    ExecuteAtomicSkillResult,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def dist(first, second):
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


def goto_yaw(start, target, initial_yaw, mode):
    if mode in ("", "fixed"):
        return initial_yaw
    if mode == "path_tangent":
        dx, dy = target[0] - start[0], target[1] - start[1]
        if math.hypot(dx, dy) <= 1e-6:
            return initial_yaw
        return math.atan2(dy, dx)
    raise ValueError("GOTO_WORLD yaw_mode must be empty, fixed or path_tangent")


class Server(object):
    def __init__(self):
        gp = rospy.get_param
        self.odom_topic = gp("~odom_topic", "/ekf/ekf_odom")
        self.frame = gp("~world_frame", "world")
        self.execution_enabled = gp("~execution_enabled", False)
        self.odom_timeout = gp("~odom_timeout", 0.5)
        self.altitude_reference_z = gp("~altitude_reference_z_m", 0.0)
        self.jump = gp("~max_localization_jump", 1.0)
        self.step = gp("~max_goal_step", 0.8)
        self.max_world_goto_distance = gp("~max_world_goto_distance", 2.0)
        self.chord = gp("~orbit_chord_length", 0.5)
        self.ptol = gp("~pos_tolerance", 0.22)
        self.pass_tol = gp("~pass_tolerance", 0.45)
        self.vtol = gp("~velocity_tolerance", 0.18)
        self.ytol = gp("~yaw_tolerance", 0.07)
        self.yrtol = gp("~yaw_rate_tolerance", 0.12)
        self.settle = gp("~settle_time", 0.4)
        self.ground = gp("~virtual_ground", 0.2)
        self.ceil = gp("~virtual_ceil", 2.8)
        self.bmin = gp("~map_min", [-1e6, -1e6, self.ground])
        self.bmax = gp("~map_max", [1e6, 1e6, self.ceil])
        self.odom = None
        self.received = None
        self.last_pos = None
        self.localization_bad = False
        self.yaw_target = None
        self.goal_pub = rospy.Publisher(gp("~goal_topic", "/goal"), PoseStamped, queue_size=1)
        self.yaw_pub = rospy.Publisher(
            gp("~yaw_topic", "/planning/yaw"), PositionCommand, queue_size=1
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / gp("~yaw_publish_rate", 20.0)), self.publish_yaw)
        self.server = actionlib.SimpleActionServer(
            "~execute", ExecuteAtomicSkillAction, self.execute, False
        )
        rospy.set_param("~goto_yaw_modes", ["fixed", "path_tangent"])
        self.server.start()
        rospy.loginfo(
            "atomic skill executor ready; odom=%s execution_enabled=%s",
            self.odom_topic,
            self.execution_enabled,
        )

    def on_odom(self, message):
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        if self.last_pos is not None and dist(position, self.last_pos) > self.jump:
            self.localization_bad = True
            rospy.logerr("localization jump detected: %.3f m", dist(position, self.last_pos))
        self.last_pos = position
        self.odom = message
        self.received = rospy.Time.now()

    def publish_yaw(self, _event):
        if self.yaw_target is not None:
            message = PositionCommand()
            message.header.stamp = rospy.Time.now()
            message.yaw = self.yaw_target
            self.yaw_pub.publish(message)

    def pose(self):
        position = self.odom.pose.pose.position
        return position.x, position.y, position.z

    def velocity(self):
        twist = self.odom.twist.twist
        return math.sqrt(
            twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2
        )

    def valid_odom(self):
        return (
            self.odom is not None
            and not self.localization_bad
            and (rospy.Time.now() - self.received).to_sec() <= self.odom_timeout
        )

    def safe(self, point):
        agl = point[2] - self.altitude_reference_z
        return (
            self.bmin[0] <= point[0] <= self.bmax[0]
            and self.bmin[1] <= point[1] <= self.bmax[1]
            and self.ground <= agl <= self.ceil
        )

    def publish_goal(self, point):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = point
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def feedback(self, status, index, count, position_error=0.0, yaw_error=0.0):
        feedback = ExecuteAtomicSkillFeedback(
            status=status,
            progress=float(index) / max(1, count),
            waypoint_index=index,
            waypoint_count=count,
            position_error=position_error,
            yaw_error=yaw_error,
        )
        self.server.publish_feedback(feedback)

    def fail(self, status, message):
        self.yaw_target = None
        self.server.set_aborted(
            ExecuteAtomicSkillResult(False, status, message), message
        )

    def wait_point(self, point, yaw, deadline, index, count, final,
                   publish_position=True):
        self.yaw_target = wrap(yaw)
        if publish_position:
            self.publish_goal(point)
        stable_since = None
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.yaw_target = None
                self.server.set_preempted(
                    ExecuteAtomicSkillResult(False, "PREEMPTED", "cancelled")
                )
                return False
            if not self.valid_odom():
                self.fail("FAILED_LOCALIZATION", "odometry stale or localization jumped")
                return False
            if rospy.Time.now() > deadline:
                self.fail("FAILED_TIMEOUT", "skill timed out")
                return False
            position_error = dist(self.pose(), point)
            yaw_error = abs(
                wrap(self.yaw_target - yaw_of(self.odom.pose.pose.orientation))
            )
            self.feedback(
                "SETTLING" if final else "EXECUTING",
                index,
                count,
                position_error,
                yaw_error,
            )
            reached = position_error < (self.ptol if final else self.pass_tol)
            if not final and reached:
                return True
            if (
                final
                and reached
                and self.velocity() < self.vtol
                and yaw_error < self.ytol
                and abs(self.odom.twist.twist.angular.z) < self.yrtol
            ):
                stable_since = stable_since or rospy.Time.now()
                if (rospy.Time.now() - stable_since).to_sec() >= self.settle:
                    return True
            else:
                stable_since = None
            rate.sleep()

    def move_points(self, start, yaw, direction, distance):
        vectors = {
            "forward": (1, 0, 0),
            "back": (-1, 0, 0),
            "left": (0, 1, 0),
            "right": (0, -1, 0),
            "up": (0, 0, 1),
            "down": (0, 0, -1),
        }
        if direction not in vectors or distance <= 0:
            raise ValueError("invalid MOVE direction or distance")
        count = max(1, int(math.ceil(distance / self.step)))
        points = []
        for index in range(1, count + 1):
            x, y, z = vectors[direction]
            amount = distance * index / count
            dx = math.cos(yaw) * x * amount - math.sin(yaw) * y * amount
            dy = math.sin(yaw) * x * amount + math.cos(yaw) * y * amount
            points.append((start[0] + dx, start[1] + dy, start[2] + z * amount))
        return points

    def world_points(self, start, target):
        distance = dist(start, target)
        if distance <= 0.01:
            raise ValueError("GOTO_WORLD target is already reached")
        if distance > self.max_world_goto_distance + 1e-6:
            raise ValueError(
                "GOTO_WORLD distance %.3f exceeds %.3f m stage limit"
                % (distance, self.max_world_goto_distance)
            )
        count = max(1, int(math.ceil(distance / self.step)))
        return [
            tuple(
                start[axis] + (target[axis] - start[axis]) * index / count
                for axis in range(3)
            )
            for index in range(1, count + 1)
        ]

    def orbit_points(self, goal, start, yaw):
        if min(goal.radius, goal.orbit_angle) <= 0 or goal.direction not in ("cw", "ccw"):
            raise ValueError("invalid ORBIT parameters")
        center = (goal.center.x, goal.center.y, goal.center.z)
        if goal.center_frame == "relative_body":
            center = (
                start[0] + math.cos(yaw) * center[0] - math.sin(yaw) * center[1],
                start[1] + math.sin(yaw) * center[0] + math.cos(yaw) * center[1],
                start[2] + center[2],
            )
        elif goal.center_frame != "world":
            raise ValueError("invalid center_frame")
        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        angle_step = 2 * math.asin(min(1.0, self.chord / (2 * goal.radius)))
        count = max(1, int(math.ceil(goal.orbit_angle / angle_step)))
        sign = 1 if goal.direction == "ccw" else -1
        points = []
        entry = (
            center[0] + goal.radius * math.cos(start_angle),
            center[1] + goal.radius * math.sin(start_angle),
            center[2],
        )
        if dist(start, entry) > 0.05:
            points.append(entry)
        for index in range(1, count + 1):
            angle = start_angle + sign * goal.orbit_angle * index / count
            points.append(
                (
                    center[0] + goal.radius * math.cos(angle),
                    center[1] + goal.radius * math.sin(angle),
                    center[2],
                )
            )
        return center, points

    def execute(self, goal):
        if not self.execution_enabled:
            self.fail("FAILED_DISABLED", "execution is disabled by safety gate")
            return
        if not self.valid_odom():
            self.fail("FAILED_LOCALIZATION", "fresh odometry required")
            return
        self.localization_bad = False
        start = self.pose()
        initial_yaw = yaw_of(self.odom.pose.pose.orientation)
        timeout = goal.timeout if goal.timeout > 0 else 60.0
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        skill = goal.skill.upper()
        try:
            if skill == "MOVE":
                points = self.move_points(
                    start, initial_yaw, goal.direction.lower(), goal.distance
                )
                yaws = [initial_yaw] * len(points)
            elif skill == "GOTO_WORLD":
                if goal.center_frame != "world":
                    raise ValueError("GOTO_WORLD requires center_frame=world")
                target = (goal.center.x, goal.center.y, goal.center.z)
                if not all(math.isfinite(value) for value in target):
                    raise ValueError("GOTO_WORLD target must be finite")
                points = self.world_points(start, target)
                yaws = [goto_yaw(start, target, initial_yaw, goal.yaw_mode)] * len(points)
            elif skill == "ROTATE":
                if goal.direction.lower() not in ("left", "right") or goal.angle <= 0:
                    raise ValueError("invalid ROTATE parameters")
                points = [start]
                yaws = [
                    wrap(
                        initial_yaw
                        + (1 if goal.direction.lower() == "left" else -1) * goal.angle
                    )
                ]
            elif skill == "HOLD":
                points = [start]
                yaws = [initial_yaw]
            elif skill == "ORBIT":
                center, points = self.orbit_points(goal, start, initial_yaw)
                yaws = []
                for point in points:
                    radial = math.atan2(point[1] - center[1], point[0] - center[0])
                    if goal.yaw_mode == "face_center":
                        yaws.append(wrap(radial + math.pi))
                    elif goal.yaw_mode == "tangent":
                        offset = math.pi / 2 if goal.direction == "ccw" else -math.pi / 2
                        yaws.append(wrap(radial + offset))
                    elif goal.yaw_mode == "fixed":
                        yaws.append(initial_yaw)
                    else:
                        raise ValueError("invalid yaw_mode")
            else:
                raise ValueError(
                    "skill must be MOVE, GOTO_WORLD, ROTATE, ORBIT or HOLD"
                )
            if not points or any(not self.safe(point) for point in points):
                raise ValueError("target outside safety bounds")
        except ValueError as exc:
            self.fail("FAILED_PLANNING", str(exc))
            return
        publish_position = skill not in ("ROTATE", "HOLD")
        for index, (point, yaw) in enumerate(zip(points, yaws), 1):
            if not self.wait_point(
                point,
                yaw,
                deadline,
                index,
                len(points),
                index == len(points),
                publish_position,
            ):
                return
        self.feedback("SUCCEEDED", len(points), len(points))
        self.server.set_succeeded(
            ExecuteAtomicSkillResult(True, "SUCCEEDED", "skill completed")
        )


if __name__ == "__main__":
    rospy.init_node("atomic_skill_executor")
    Server()
    rospy.spin()
