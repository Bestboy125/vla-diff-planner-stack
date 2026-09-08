#!/usr/bin/env python3
"""Exercise actual atomic execution logic offline, without ROS initialization."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


source = Path(__file__).with_name("atomic_skill_server.py")
tree = ast.parse(source.read_text(encoding="utf-8"))
# Import the actual definitions, excluding ROS imports and the executable main.
definitions = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
namespace = {
    "math": math,
    "rospy": NS(Time=NS(now=lambda: 0.0), Duration=lambda value: value,
                Rate=lambda hz: Mock(), is_shutdown=lambda: False),
    "ExecuteAtomicSkillResult": lambda *args: args,
}
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), namespace)
Server = namespace["Server"]


class ScanYawTests(unittest.TestCase):
    def server(self, yaw=0.7):
        server = Server.__new__(Server)
        server.execution_enabled = True
        server.valid_odom = lambda: True
        server.pose = lambda: (0.0, 0.0, 1.0)
        server.odom = NS(pose=NS(pose=NS(orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
        server.step = 0.8
        server.max_world_goto_distance = 2.0
        server.safe = lambda point: True
        server.wait_point = Mock(return_value=True)
        server.feedback = Mock()
        server.fail = Mock()
        server.server = Mock()
        return server

    def goal(self, x, y, mode=""):
        return NS(skill="GOTO_WORLD", center_frame="world", center=NS(x=x, y=y, z=1.0),
                  yaw_mode=mode, timeout=60.0)

    def test_scan_faces_each_segment_in_world_despite_initial_heading(self):
        for x, y in ((1, 0), (-1, 0), (0, 1), (0.6, 0.4), (-0.6, 0.4)):
            for initial in (0.7, -3.13, 3.13):
                server = self.server(initial)
                server.execute(self.goal(x, y, "path_tangent"))
                calls = server.wait_point.call_args_list
                for call in calls:
                    self.assertAlmostEqual(call.args[1], math.atan2(y, x))
                    self.assertTrue(call.args[6])
                self.assertEqual(calls[-1].args[0], (x, y, 1.0))
                server.server.set_succeeded.assert_called_once()

    def test_legacy_goto_keeps_yaw_without_alignment(self):
        for mode in ("", "fixed"):
            server = self.server()
            server.execute(self.goal(1, 0, mode))
            for call in server.wait_point.call_args_list:
                self.assertAlmostEqual(call.args[1], 0.7)
                self.assertTrue(call.args[6])
            self.assertNotEqual(server.wait_point.call_args_list[0].args[0], (0, 0, 1))

    def test_failed_wait_does_not_dispatch_more_points_or_succeed(self):
        server = self.server()
        server.wait_point.return_value = False
        server.execute(self.goal(1, 0, "path_tangent"))
        self.assertEqual(server.wait_point.call_count, 1)
        server.server.set_succeeded.assert_not_called()

    def test_invalid_mode_or_oversize_leg_rejected(self):
        for goal in (self.goal(1, 0, "typo"), self.goal(3, 0, "path_tangent")):
            server = self.server()
            server.execute(goal)
            server.fail.assert_called_once()
            server.wait_point.assert_not_called()

    def test_cancellation_and_failure_release_yaw(self):
        server = self.server()
        server.server.is_preempt_requested.return_value = True
        self.assertFalse(Server.wait_point(server, (0, 0, 1), 1.2, 60, 1, 1,
                                           True, publish_position=False))
        self.assertIsNone(server.yaw_target)
        server.yaw_target = 1.2
        Server.fail(server, "FAILED_TIMEOUT", "timeout")
        self.assertIsNone(server.yaw_target)


if __name__ == "__main__":
    unittest.main()
