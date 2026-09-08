#!/usr/bin/env python3
"""Offline actual-callback tests. No ROS connections or vehicle commands."""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from collections import deque
import numpy as np
from semantic_orbit_contract import (SemanticOrbitError, build_staged_orbit_spec,
                                     validate_semantic_orbit_request)

ROOT = Path(__file__).parent
class Stamp(float):
    def __sub__(self, other):
        return Stamp(float(self)-float(other))
    def to_sec(self):
        return float(self)
    def is_zero(self):
        return self == 0

def point():
    return NS(header=NS(stamp=Stamp(10), frame_id="world"), point=NS(x=0,y=0,z=0))

env = dict(math=math, json=json, SemanticOrbitError=SemanticOrbitError,
           build_staged_orbit_spec=build_staged_orbit_spec,
           validate_semantic_orbit_request=validate_semantic_orbit_request,
           rospy=NS(Time=NS(now=lambda: Stamp(10)), Duration=lambda x:x),
           PointStamped=point, ExecuteAtomicSkillGoal=lambda: NS(center=NS()),
           GoalStatus=NS(SUCCEEDED=3))
tree = ast.parse((ROOT/'semantic_orbit_executor_node.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)],
                        type_ignores=[]), '<executor>', 'exec'), env)
Executor = env['SemanticOrbitExecutor']

def request(**extra):
    return dict(task_id='test', target_label='chair', radius_m=1.5, laps=1,
                direction='clockwise', yaw_mode='face_center',
                keep_current_altitude=True, **extra)

class RefinementTests(unittest.TestCase):
    def executor(self):
        e=Executor.__new__(Executor)
        e.lock=threading.RLock()
        e.active=validate_semantic_orbit_request(request())
        e.class_ready=True
        e.phase='DETECTING'
        e.world_frame='world'
        e.observation_not_before=Stamp(9)
        e.target_timeout=.5
        e.locked_target=None
        e.target_match_radius=1.0
        e.max_approach_leg=2
        e.require_flight_ready=Mock(return_value=(0,0,1))
        e.client=Mock()
        e.entry_world_pub=Mock()
        e.publish_status=Mock()
        e.start_approach=Mock()
        e.start_orbit=Mock()
        e.finish_rejected=Mock()
        return e

    def target(self,x,stamp=10):
        m=point(); m.point.x=x; m.point.z=1; m.header.stamp=Stamp(stamp)
        return m

    def test_hint_validation(self):
        self.assertEqual(validate_semantic_orbit_request(request(target_world_hint=[12,0,1]))
                         ['target_world_hint'],(12,0,1))
        for bad in ([1,2], [1,2,float('nan')], 'xyz'):
            with self.assertRaises(SemanticOrbitError):
                validate_semantic_orbit_request(request(target_world_hint=bad))

    def test_far_coarse_approach_limited(self):
        e=self.executor(); e.on_coarse_target(self.target(12))
        spec=e.start_approach.call_args.args[2]
        self.assertEqual(spec['approach_waypoint_world'],(2,0,1))
        e.start_orbit.assert_not_called()

    def test_coarse_cannot_orbit_even_near_entry(self):
        e=self.executor(); e.on_coarse_target(self.target(3))
        e.start_orbit.assert_not_called(); e.start_approach.assert_not_called()
        self.assertEqual(e.publish_status.call_args.args[0],'WAITING_FOR_PRECISE_TARGET')

    def test_refined_near_can_orbit(self):
        e=self.executor(); e.locked_target=(3.3,0,1)
        e.on_stable_target(self.target(3))
        e.start_orbit.assert_called_once()
        self.assertEqual(e.locked_target,(3,0,1))

    def test_other_chair_and_stale_no_motion(self):
        e=self.executor(); e.locked_target=(12,0,1)
        e.on_coarse_target(self.target(16))
        e.start_approach.assert_not_called(); e.start_orbit.assert_not_called()
        self.assertEqual(e.locked_target,(12,0,1))
        e.on_coarse_target(self.target(12,stamp=8))
        e.start_approach.assert_not_called()

    def test_duplicate_class_ack_cannot_interrupt_approach(self):
        e=self.executor(); e.phase='APPROACHING'
        e.on_target_class_status(NS(data='chair'))
        self.assertEqual(e.phase,'APPROACHING')

    def test_success_reports_actual_center(self):
        e=self.executor(); e.orbit_target=(3,0,.8)
        e.on_skill_done(e.active,3,NS(success=True,message='done'))
        self.assertEqual(e.publish_status.call_args.args[3]['target_world'],[3,0,.8])

    def test_real_localizer_stability_band_routing(self):
        # Execute the actual stability block, with observation-only publishers.
        tree=ast.parse((ROOT/'semantic_raw_stereo_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and
                 any(isinstance(f,ast.FunctionDef) and f.name=='process' for f in n.body))
        process=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='process')
        start=next(i for i,n in enumerate(process.body) if isinstance(n,ast.Assign)
                   and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='coarse')
        end=next(i for i,n in enumerate(process.body) if isinstance(n,ast.Assign)
                 and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='payload')
        block=compile(ast.Module(body=process.body[start:end],type_ignores=[]),'<stability>','exec')
        s=NS(precise_depth=6,last_depth_band=None,target_history=deque(maxlen=4),
             min_stable_observations=4,max_target_jitter=.35,standoff=1,
             keep_body_altitude=True,stable_pub=Mock(),stable_world_pub=Mock(),
             coarse_world_pub=Mock(),lock=threading.RLock(),goal_pub=Mock(),
             auto_publish_stable_goal=True,last_auto_goal=None)
        context=dict(self=s,np=np,deepcopy=deepcopy,PointStamped=point,
                     result={'depth_m':12},point_world=np.array([12.,0,1]),
                     candidate=NS(pose=NS(position=NS())),target_world_message=point(),
                     body_position=np.array([0.,0,1]),now=Stamp(10),
                     standoff_goal=lambda *a,**kw:np.array([2.,0,1]))
        for _ in range(4): exec(block,context)
        s.coarse_world_pub.publish.assert_called_once()
        s.stable_world_pub.publish.assert_not_called(); s.goal_pub.publish.assert_not_called()
        context['result']['depth_m']=5
        for _ in range(3): exec(block,context)
        s.stable_world_pub.publish.assert_not_called()
        exec(block,context)
        s.stable_world_pub.publish.assert_called_once()

if __name__=='__main__': unittest.main()
