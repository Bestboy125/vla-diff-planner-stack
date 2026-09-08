#!/usr/bin/env python3
"""Offline tests of the actual mission request and dispatch methods."""
import ast
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from scan_route_geometry import ScanMissionError, generate_world_serpentine, validate_request

source = Path(__file__).with_name("semantic_scan_orbit_node.py")
tree = ast.parse(source.read_text(encoding="utf-8"))
definitions = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
params = {"/atomic_skill_executor/goto_yaw_modes": ["fixed", "path_tangent"],
          "/drone_0_traj_server/supports_yaw_settle": True}
namespace = dict(math=math, json=json, time=time, ScanMissionError=ScanMissionError,
                 generate_world_serpentine=generate_world_serpentine,
                 validate_request=validate_request,
                 rospy=NS(Time=Mock(return_value=0, now=lambda: 1),
                          Duration=lambda v: v, get_param=lambda key, default: params.get(key, default)),
                 String=lambda data: NS(data=data),
                 ExecuteAtomicSkillGoal=lambda: NS(center=NS()))
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), namespace)
Mission = namespace["SemanticScanOrbitMission"]


def odom(x, y, z, yaw):
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y, z=z), orientation=NS(
        x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2)))))


class MissionTests(unittest.TestCase):
    def mission(self, yaw):
        m = Mission.__new__(Mission)
        m.lock = threading.RLock()
        m.execution_enabled = True
        m.active = None
        m.scan_width, m.scan_length, m.sweep_count = 6, 10, 5
        m.waypoint_spacing, m.turn_samples, m.turn_bulge_ratio = 1, 6, 0.45
        m.world_frame = "world"
        m.max_goto_leg, m.waypoint_timeout = 1.8, 60
        m.require_flight_ready = Mock(return_value=odom(10, 20, 1.2, yaw))
        m.client = Mock()
        m.target_class_pub = Mock()
        m.target_class_pub.get_num_connections.return_value = 1
        m.semantic_orbit_request_pub = Mock()
        m.semantic_orbit_request_pub.get_num_connections.return_value = 1
        m.route_pub = Mock()
        m.publish_status = Mock()
        request = dict(task_id="test", target_label="chair", scan_width_m=6,
                       scan_length_m=10, sweep_count=5, radius_m=1.5, laps=1,
                       direction="clockwise", yaw_mode="face_center", keep_current_altitude=True)
        m.on_request(NS(data=json.dumps(request)))
        return m

    def test_fixed_axes_independent_of_body_yaw(self):
        for yaw in (0, 0.7, math.pi/2, -math.pi):
            m = self.mission(yaw)
            self.assertEqual(m.route[0]["world"], (11, 20, 1.2))
            self.assertEqual(m.route[-1]["world"], (16, 30, 1.2))
            payload = json.loads(m.route_pub.publish.call_args.args[0].data)
            self.assertEqual(payload["scan_heading_rad"], 0)
            m.dispatch_route_leg()
            goal = m.client.send_goal.call_args.args[0]
            self.assertEqual((goal.center.x, goal.center.y), (11, 20))
            self.assertEqual(goal.yaw_mode, "path_tangent")

    def test_orbit_return_keeps_route_and_limits_leg(self):
        m = self.mission(0.3)
        original = list(m.route)
        m.route_index = 8
        m.require_flight_ready.return_value = odom(-3, -4, 1.2, 2)
        m.dispatch_route_leg()
        self.assertEqual(m.route, original)
        self.assertEqual(m.route_index, 8)
        self.assertEqual(m.phase, "RETURNING")
        goal = m.client.send_goal.call_args.args[0]
        self.assertAlmostEqual(math.hypot(goal.center.x+3, goal.center.y+4), 1.8)
        self.assertEqual(goal.yaw_mode, "path_tangent")

    def test_old_binary_rejected_before_motion(self):
        params["/drone_0_traj_server/supports_yaw_settle"] = False
        try:
            m = self.mission(0)
            self.assertIsNone(m.active)
            m.client.send_goal.assert_not_called()
            self.assertEqual(m.publish_status.call_args.args[0], "REJECTED")
        finally:
            params["/drone_0_traj_server/supports_yaw_settle"] = True

    def test_orbit_request_carries_lock_and_success_commits_refined_target(self):
        m = self.mission(0)
        m.pending_target = (12, 0, .8)
        m.orbit_radius, m.orbit_laps = 1.5, 1
        m.orbit_direction, m.orbit_yaw_mode = 'clockwise', 'face_center'
        m.body_frame = 'base_link'
        m.request_stereo_orbit()
        payload = json.loads(m.semantic_orbit_request_pub.publish.call_args.args[0].data)
        self.assertEqual(payload['target_world_hint'], [12, 0, .8])
        self.assertEqual(m.processed_targets, [])
        m.post_orbit_cooldown = 5
        m.dispatch_route_leg = Mock()
        m.on_orbit_status(NS(data=json.dumps(dict(task_id=m.active_orbit_task_id,
                                                state='SUCCEEDED', target_world=[11.6,0,.8]))))
        self.assertEqual(m.processed_targets, [(11.6, 0, .8)])
        m.dispatch_route_leg.assert_called_once()

    def test_failed_orbit_does_not_mark_processed(self):
        m = self.mission(0)
        m.phase = 'ORBIT_REQUESTED'
        m.active_orbit_task_id = 'orbit-fail'
        m.fail = Mock()
        m.on_orbit_status(NS(data=json.dumps(dict(task_id='orbit-fail', state='FAILED'))))
        self.assertEqual(m.processed_targets, [])
        m.fail.assert_called_once()


if __name__ == "__main__":
    unittest.main()
