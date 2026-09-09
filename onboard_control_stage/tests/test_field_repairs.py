"""Offline only: actual methods with ROS and publishers mocked."""
import ast
import __future__
import json
import math
import threading
import time
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

def load(path, name, extra=None):
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    definitions = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                   and (not isinstance(n, ast.ClassDef) or n.name == name)]
    ns = dict(math=math, time=time, json=json, threading=threading,
              rospy=NS(Time=NS(now=lambda: 100)), GoalStatus=NS(SUCCEEDED=3),
              ScanMissionError=RuntimeError, ProtocolError=RuntimeError,
              String=lambda data: NS(data=data), Empty=lambda: NS(), GoalID=lambda: NS(),
              ExecuteAtomicSkillGoal=lambda: NS(center=NS()))
    ns.update(extra or {})
    exec(compile(ast.Module(body=definitions,type_ignores=[]), str(path), 'exec',
                 flags=__future__.annotations.compiler_flag), ns)
    return ns[name]

ROOT = Path(__file__).resolve().parents[2]
SCAN_DIR = ROOT / 'onboard_scan_stage/semantic_scan_orbit_mission/scripts'
sys.path.insert(0, str(SCAN_DIR))
Scan = load(SCAN_DIR / 'semantic_scan_orbit_node.py', 'SemanticScanOrbitMission')
Bridge = load(ROOT / 'Diff-Planner/src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py', 'VlaDiffBridge')

class FieldRepairs(unittest.TestCase):
    def scan(self):
        s=Scan.__new__(Scan); s.lock=threading.RLock(); s.active={'task_id':'scan'}
        s.phase='NAVIGATING'; s.route=[dict(world=(i,0,1),kind='line',sweep_index=0) for i in (1,2,3)]
        s.route_index=0; s.max_goto_leg=1.8; s.waypoint_timeout=60
        s.client=Mock(); s.publish_status=Mock(); s.hover_pub=Mock()
        s.require_flight_ready=Mock(return_value=NS(pose=NS(pose=NS(position=NS(x=0,y=0,z=1)))))
        s.processed_targets=[]; s.active_orbit_task_id=None
        s.dispatch_route_leg()
        return s

    def test_scan_three_points_not_reentrant(self):
        s=self.scan()
        self.assertNotIn('done_cb',s.client.send_goal.call_args.kwargs)
        for index in range(3):
            # Terminal wire state alone must NOT dispatch before actionlib DONE.
            s.client.simple_state=1; s.client.get_state.return_value=3
            before=s.client.send_goal.call_count; s.on_timer(None)
            self.assertEqual(s.client.send_goal.call_count,before)
            s.require_flight_ready.return_value.pose.pose.position.x=index+1
            s.client.simple_state=2; s.client.get_result.return_value=NS(success=True)
            s.on_timer(None)
        self.assertIsNone(s.active)
        self.assertEqual(s.client.send_goal.call_count,3)

    def test_scan_cancel_prevents_resumption(self):
        s=self.scan(); s.on_cancel(None); count=s.client.send_goal.call_count
        s.client.simple_state=2; s.on_timer(None)
        self.assertEqual(s.client.send_goal.call_count,count)
        s.client.cancel_goal.assert_called_once(); s.hover_pub.publish.assert_called_once()

    def test_scan_timeout(self):
        s=self.scan(); s.client.simple_state=1; s.navigation_started=time.monotonic()-65
        s.on_timer(None); self.assertIsNone(s.active)
        self.assertEqual(s.publish_status.call_args.args[0],'FAILED')

    def bridge(self):
        b=Bridge.__new__(Bridge); b._publish_status=Mock(); b._takeoff=dict(command=NS(),started=time.monotonic()-2)
        return b

    def test_failed_arm_is_reported_without_retry(self):
        b=self.bridge(); b._latest_fcu_state=(True,False,time.monotonic()); b._px4_state=('MANUAL_CTRL',time.monotonic())
        b._advance_takeoff(); self.assertIsNone(b._takeoff)
        self.assertEqual(b._publish_status.call_args.args[0],'takeoff_failed')

    def test_takeoff_requires_armed_hover_for_completion(self):
        b=self.bridge(); b._latest_fcu_state=(True,True,time.monotonic()); b._px4_state=('AUTO_TAKEOFF',time.monotonic())
        b._advance_takeoff(); b._publish_status.assert_not_called()
        b._px4_state=('AUTO_HOVER',time.monotonic()); b._advance_takeoff()
        self.assertEqual(b._publish_status.call_args.args[0],'takeoff_complete')

    def test_stop_cancels_atomic_and_all_executors(self):
        b=self.bridge(); b._active_semantic_task={'task_id':'scan','command':'SEMANTIC_SCAN_ORBIT'}
        for key in ('atomic_cancel_pub','semantic_orbit_cancel_pub','monocular_semantic_orbit_cancel_pub',
                    'semantic_scan_orbit_cancel_pub','hybrid_semantic_orbit_cancel_pub','hover_pub'):
            setattr(b,key,Mock())
        b._cancel_motion_sources()
        self.assertIsNone(b._active_semantic_task); self.assertIn('scan',b._cancel_pending)
        b.atomic_cancel_pub.publish.assert_called_once(); b.hover_pub.publish.assert_called_once()

    def test_landing_disarm_delivery_order(self):
        b=self.bridge(); b._landing=dict(command=NS(),phase='LANDING',started=time.monotonic())
        b._latest_fcu_state=(True,True,time.monotonic()); b._px4_state=('MANUAL_CTRL',time.monotonic())
        b._advance_landing(); b._publish_status.assert_not_called()
        b._latest_fcu_state=(True,False,time.monotonic()); b._advance_landing()
        self.assertEqual(b._publish_status.call_args.args[0],'land_complete')

if __name__ == '__main__': unittest.main()
