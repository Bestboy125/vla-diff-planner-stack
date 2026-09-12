"""No ROS master, camera, planner or flight-controller is contacted by these tests."""
import ast
import json
import math
import pathlib
import sys
import threading
import time
import types
from unittest.mock import Mock
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'onboard_target_stage/target_interaction/scripts'
sys.path.insert(0, str(SCRIPTS))
from target_geometry import validate, approach_goal, retreat_goal, bounded_goal, yaw_error, bearing
from target_geometry import pass_route, landing_candidates, target_axes, segment_distance, FullRotation

def spec(action='APPROACH'):
    return dict(task_id='test', action=action, target_label='chair', selector='left',
                stand_off_m=1.5, object_radius_m=.5, distance_m=1., max_travel_m=10.)

@pytest.mark.parametrize('angle',[0,math.pi/2,math.pi,-math.pi/2])
def test_geometry_world_directions(angle):
    p=(0,0,1); c=(5*math.cos(angle),5*math.sin(angle),0)
    goal=approach_goal(p,c,2)
    assert math.dist(p,goal)==pytest.approx(1)
    assert goal[2]==1
    assert math.dist(retreat_goal(p,c,1)[:2],c[:2])==pytest.approx(6)
    assert bearing(p,c)==pytest.approx(angle)

@pytest.mark.parametrize('key,value',[('distance_m',float('nan')),('distance_m',True),('selector','above'),('target_label','two chairs'),('max_travel_m',16)])
def test_contract_rejects_invalid(key,value):
    s=spec(); s[key]=value
    with pytest.raises(ValueError): validate(s)

def test_clearance_budget_and_wrap():
    assert approach_goal((0,0,1),(2,0,1),2) is None
    assert bounded_goal((0,0,1),(3,0,1))==(1,0,1)
    assert yaw_error(-math.pi+.1,math.pi-.1)==pytest.approx(.2)
    s=spec('RETREAT'); s.update(distance_m=2,max_travel_m=1)
    with pytest.raises(ValueError): validate(s)

class Message:
    def __init__(self,data=None):
        self.data=data; self.header=types.SimpleNamespace(stamp=None,frame_id='')
        self.position=types.SimpleNamespace(x=0,y=0,z=0)
        self.center=types.SimpleNamespace(x=0,y=0,z=0)

class Stamp(float):
    def to_sec(self): return float(self)
    def __sub__(self,other): return Stamp(float(self)-float(other))

def node():
    tree=ast.parse((SCRIPTS/'target_interaction_node.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    clock=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:Stamp(100)))
    env=dict(json=json,math=math,time=time,rospy=clock,String=Message,Empty=Message,
             PositionCommand=Message,ExecuteAtomicSkillGoal=Message,
             GoalStatus=types.SimpleNamespace(SUCCEEDED=3),validate=validate,bearing=bearing,
             yaw_error=yaw_error,approach_goal=approach_goal,retreat_goal=retreat_goal,bounded_goal=bounded_goal)
    env.update(pass_route=pass_route,landing_candidates=landing_candidates,target_axes=target_axes,
               segment_distance=segment_distance,FullRotation=FullRotation)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'<target node>','exec'),env)
    n=env['TargetInteraction'].__new__(env['TargetInteraction'])
    n.active=spec(); n.lock=threading.RLock(); n.phase='DETECTING'; n.ready=True
    n.start=n.last_position=(0,0,1); n.target=(5,0,1); n.travel=0; n.goal_world=None
    n.candidate=((5,0,1),False,Stamp(100)); n.settled=None; n.retreat_end=None
    n.client=Mock(); n.client.simple_state=1
    n.status_pub=Mock(); n.hover_pub=Mock(); n.class_pub=Mock(); n.rotation_pub=Mock()
    n.flight=Mock(return_value=((0,0,1),0,0,0)); n.fcu=None
    n.started=n.phase_started=time.monotonic(); n.last_status=0
    n.observation_after=Stamp(99)
    return n

def test_approach_one_stage_no_callback():
    n=node(); n.advance_target((0,0,1),0,0,0)
    goal=n.client.send_goal.call_args.args[0]
    assert goal.skill=='GOTO_WORLD' and goal.center.x==1 and goal.center.z==1
    assert not n.client.send_goal.call_args.kwargs

def test_coarse_depth_cannot_finish_near_target():
    n=node(); n.candidate=((3,0,1),True,Stamp(100))
    n.advance_target((0,0,1),0,0,0)
    n.client.send_goal.assert_not_called(); assert n.active

def test_action_done_reacquires_instead_of_replaying():
    n=node(); n.phase='MOVING'; n.client.simple_state=2
    n.client.get_state.return_value=3; n.client.get_result.return_value=types.SimpleNamespace(success=True)
    n.tick(None)
    assert n.phase=='DETECTING' and n.candidate is None
    assert json.loads(n.class_pub.publish.call_args.args[0].data)['selector']=='highest_confidence'
    n.client.send_goal.assert_not_called()

def test_other_target_or_stale_target_is_ignored():
    n=node(); n.candidate=None
    msg=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='world',stamp=Stamp(100)),point=types.SimpleNamespace(x=10,y=0,z=1))
    n.on_target(msg,False); assert n.candidate is None
    msg.point.x=5; msg.header.stamp=Stamp(98)
    n.on_target(msg,False); assert n.candidate is None
    msg.header.stamp=Stamp(100); n.on_target(msg,False); assert n.candidate

def test_turn_uses_hold_lease_not_unseeded_atomic_rotate():
    n=node(); n.active=spec('TURN'); n.target=(0,5,1)
    n.advance_target((0,0,1),0,0,0)
    msg=n.rotation_pub.publish.call_args.args[0]
    assert msg.position.z==1 and msg.yaw==pytest.approx(math.pi/4)
    n.client.send_goal.assert_not_called(); assert n.phase=='FACING'
    n.start=(1,0,1)
    with pytest.raises(ValueError,match='drift'): n.advance_target((0,0,1),0,0,0)

def test_stop_waits_for_action_and_measured_settling():
    n=node(); n.phase='MOVING'; n.cancel(None)
    assert n.phase=='STOPPING' and n.active
    n.confirm_stop(); assert n.active
    n.client.simple_state=2; n.confirm_stop(); n.settled=time.monotonic()-1
    n.confirm_stop(); assert n.active is None
    assert json.loads(n.status_pub.publish.call_args.args[0].data)['state']=='CANCELLED'

def test_failed_action_is_not_skipped():
    n=node(); n.phase='MOVING'; n.client.simple_state=2; n.client.get_state.return_value=4
    n.tick(None); assert n.phase=='STOPPING'
    n.client.send_goal.assert_not_called()

def test_region_crop_restores_full_image_coordinates():
    import numpy as np
    path=ROOT/'onboard_semantic_stage/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='SemanticRawStereoNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='select_detection')
    env={}; exec(compile(ast.Module(body=[method],type_ignores=[]),'<selector>','exec'),env)
    n=types.SimpleNamespace(target_selector='right',debug_bbox=None,_select_detection=Mock(return_value=(.9,[10,10,50,50],'mock')))
    result=env['select_detection'](n,np.zeros((100,300,3)),None)
    assert result[1]==[210,10,250,50]
    assert n._select_detection.call_args.args[0].shape==(100,100,3)

@pytest.mark.parametrize('failure',['disarmed','stale_odom','wrong_frame','manual','altitude'])
def test_real_flight_gate_rejects_unsafe_state(failure):
    n=node(); S=types.SimpleNamespace
    n.reference=0.; n.min_altitude=.3; n.max_altitude=2.
    odom=S(header=S(stamp=Stamp(100),frame_id='world'),
           pose=S(pose=S(position=S(x=0,y=0,z=1),orientation=S(x=0,y=0,z=0,w=1))),
           twist=S(twist=S(linear=S(x=0,y=0,z=0),angular=S(z=0))))
    n.odom=(odom,time.monotonic()); n.fcu=(S(connected=True,armed=True),time.monotonic())
    n.px4=('AUTO_HOVER',time.monotonic())
    assert n.__class__.flight(n)[0]==(0,0,1)
    if failure=='disarmed': n.fcu[0].armed=False
    if failure=='stale_odom': n.odom=(odom,time.monotonic()-1)
    if failure=='wrong_frame': odom.header.frame_id='map'
    if failure=='manual': n.px4=('MANUAL_CTRL',time.monotonic())
    if failure=='altitude': odom.pose.pose.position.z=3
    with pytest.raises(ValueError): n.__class__.flight(n)
