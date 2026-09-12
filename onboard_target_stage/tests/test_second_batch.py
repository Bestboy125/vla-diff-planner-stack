import math
import pathlib
import sys
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
import pytest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'target_interaction/scripts'))
from target_geometry import FullRotation,pass_route,target_axes,landing_candidates,validate
from landing_safety import LandingMap
from test_target_skills import node,spec

@pytest.mark.parametrize('angle',[0,math.pi/2,math.pi,-math.pi/2])
@pytest.mark.parametrize('side',['left','right'])
def test_fixed_observer_side_geometry(angle,side):
    start=(0,0,1); target=(5*math.cos(angle),5*math.sin(angle),1)
    route=pass_route(start,target,side,2,1)
    u,v=target_axes(start,target,side)
    assert all(sum((p[i]-target[i])*v[i] for i in (0,1))==pytest.approx(2) for p in route)
    assert sum((route[-1][i]-target[i])*u[i] for i in (0,1))==pytest.approx(3)
    assert all(p[2]==1 for p in route)
    assert all(sum((p[i]-target[i])*v[i] for i in (0,1))>=2-1e-9 for p in landing_candidates(start,target,side,2))

@pytest.mark.parametrize('direction',['clockwise','counterclockwise'])
@pytest.mark.parametrize('laps',[1,2,3])
def test_rotation_measured_unwrap_multiple_full_laps(direction,laps):
    rotation=FullRotation(3.1,direction,laps,30); yaw=3.1
    for _ in range(2300):
        target=rotation.step(yaw,.1)
        yaw=target
        if rotation.reached(): break
    assert rotation.reached()
    assert rotation.measured==pytest.approx(laps*2*math.pi,abs=.05)

def test_rotation_stationary_cannot_report_full_turn_and_wrong_direction_rejected():
    r=FullRotation(0,'clockwise',1,30)
    for _ in range(100): r.step(0,.1)
    assert not r.reached() and r.commanded<=math.radians(20)
    with pytest.raises(ValueError): r.step(.3,.1)
    with pytest.raises(ValueError): FullRotation(0,'clockwise',1,30).step(-1,.1)

def flat_map():
    m=LandingMap(); m.last_stamp=100
    for x in range(-8,9):
        for y in range(-8,9):
            m.cells[x,y,-1]=(1,100); m.heights[x,y,-1]=(-.1,-.1)
            for z in range(0,10): m.cells[x,y,z]=(0,100)
    return m

def test_landing_observed_flat_clear_column():
    evidence=flat_map().assess((0,0,1),1,.8,0,100)
    assert evidence['center_world'][2]==pytest.approx(-.1)
    assert evidence['ground_cells']>=81

@pytest.mark.parametrize('failure',['unknown_ground','unknown_air','obstacle','stale','rough','slope'])
def test_landing_rejects_unknown_obstacle_stale_and_uneven(failure):
    m=flat_map()
    if failure=='unknown_ground': m.cells.pop((0,0,-1))
    if failure=='unknown_air': m.cells.pop((0,0,3))
    if failure=='obstacle': m.cells[0,0,3]=(1,100)
    if failure=='stale': m.last_stamp=98
    if failure=='rough': m.heights[0,0,-1]=(-.1,0)
    if failure=='slope':
        for k in m.heights: m.heights[k]=(.04*k[0],.04*k[0])
    with pytest.raises(ValueError): m.assess((0,0,1),1,.8,0,100)

def test_rays_never_invent_unobserved_free_space_or_clear_recent_obstacle():
    m=LandingMap(); m.ingest(np.array([0,0,1]),[(2,0,1)],100,100)
    occupied=m.key((2,0,1)); assert m.cells[occupied][0]==1
    m.ingest(np.array([0,0,1]),[(3,0,1)],100.1,100.1)
    assert m.cells[occupied][0]==1
    assert m.key((1,2,1)) not in m.cells
    with pytest.raises(ValueError): m.assess((0,0,1),1,.8,0,100.1)

def test_pass_budget_checked_before_motion():
    n=node(); n.active=spec('PASS'); n.active.update(side='right',exit_distance_m=1,max_travel_m=2)
    n.route=None; n.route_index=0
    with pytest.raises(ValueError,match='budget'): n.advance_route((0,0,1),0,0,0)
    n.client.send_goal.assert_not_called()

def test_land_no_safe_site_never_moves_or_lands():
    n=node(); n.active=spec('LAND'); n.active.update(side='right',exit_distance_m=1)
    n.route=None; n.route_index=0; n.reference=0
    n.landing_observer=Mock(); n.landing_observer.assess.side_effect=ValueError('unknown ground')
    n.land_pub=Mock()
    with pytest.raises(ValueError,match='no observed'): n.advance_route((0,0,1),0,0,0)
    n.client.send_goal.assert_not_called(); n.land_pub.publish.assert_not_called()

@pytest.mark.parametrize('action,extra',[('PASS',dict(side='above')),('ROTATE',dict(rotation_laps=.5)),('LAND',dict(side='left',exit_distance_m=float('nan')))])
def test_second_batch_invalid_contract(action,extra):
    s=spec(action); s.update(side='right',rotation_direction='clockwise',rotation_laps=1,rotation_rate_deg_s=30)
    s.update(extra)
    with pytest.raises(ValueError): validate(s)

def test_pass_executes_each_bounded_leg_and_confirms_exit():
    n=node(); n.active=spec('PASS'); n.active.update(side='right',exit_distance_m=1)
    n.route=None; n.route_index=0; p=(0,0,1)
    for _ in range(30):
        if n.active is None: break
        n.last_position=p
        n.advance_route(p,0,0,0)
        if n.phase=='MOVING':
            n.check_route(p)
            p=n.goal_world; n.phase='VERIFYING'
    assert n.active is None and math.dist(p,(8,-2,1))<.16

def test_pass_opposite_side_or_corridor_deviation_stops():
    n=node(); n.active=spec('PASS'); n.active.update(side='left',exit_distance_m=1)
    n.phase='MOVING'; n.move_origin=(3,2,1); n.goal_world=(4,2,1)
    with pytest.raises(ValueError): n.check_route((4,-2,1))

def test_land_handoff_occurs_once_after_fresh_final_site_check():
    import time
    n=node(); n.active=spec('LAND'); n.active.update(side='right',exit_distance_m=1)
    n.route=[(0,0,1)]; n.route_index=0; n.reference=0
    n.landing_observer=Mock(); n.landing_observer.assess.return_value=dict(center_world=[0,0,-.1],radius_m=.8,observed_stamp=100)
    n.land_pub=Mock(); n.settled=time.monotonic()-1
    n.advance_route((0,0,1),0,0,0)
    assert n.phase=='LAND_HANDOFF'; n.land_pub.publish.assert_called_once()
    n.tick(None); n.land_pub.publish.assert_called_once()
    n.hover_pub.publish.assert_not_called()
