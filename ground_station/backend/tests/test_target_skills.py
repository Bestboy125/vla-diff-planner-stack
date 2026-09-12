"""Frontend contract -> backend -> actual bridge parser over localhost; no aircraft I/O."""
import asyncio
import json
import pathlib
import sys
import time
from unittest.mock import Mock
import pytest
from fastapi import HTTPException
from fastapi.testclient import TestClient
from app.main import app
from app.mission import MissionManager
from app.onboard_bridge import OnboardBridgeClient
from app.schemas import TaskDispatchRequest
from app.task_dispatch import TaskDispatcher

ROOT=pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Diff-Planner/src/integration/vla_diff_bridge/test'))
from test_semantic_orbit_forwarding import SemanticOrbitForwardingTest
from test_waypoint_execution import BRIDGE
from vla_diff_bridge.protocol import parse_command, ProtocolError

@pytest.mark.parametrize('task,action', [('approach_target','APPROACH'),('retreat_target','RETREAT'),('turn_to_target','TURN'),('pass_side','PASS'),('rotate_full','ROTATE'),('land_near_target','LAND')])
def test_api_dry_run(task,action):
    response=TestClient(app).post('/api/tasks/dispatch',json=dict(category='embodied',embodied_task=task,
             mode='dry_run',parameters=dict(target_label='Chair',target_selector='right')))
    assert response.status_code==200,response.text
    data=response.json(); assert data['delivery']['status']=='safety_locked'
    parsed=parse_command(data['command']); assert parsed.target_skill['action']==action
    assert parsed.target_skill['target_label']=='chair' and parsed.target_skill['selector']=='right'

@pytest.mark.parametrize('parameters',[dict(target_label='two chairs'),dict(target_label='chair',target_selector='above'),
    dict(target_label='chair',retreat_distance_m=3,max_travel_m=1),dict(target_label='chair',stand_off_m=.5)])
def test_api_rejects_bad_contract(parameters):
    response=TestClient(app).post('/api/tasks/dispatch',json=dict(category='embodied',embodied_task='retreat_target',parameters=parameters))
    assert response.status_code==422

def test_authenticated_socket_bridge_mutex_and_result_return():
    async def scenario():
        fixture=SemanticOrbitForwardingTest(); fixture.setUp(); b=fixture.bridge
        b.target_skill_pub=Mock(); b.target_skill_pub.get_num_connections.return_value=1
        b._latest_fcu_state=(True,True,time.monotonic()); b._px4_state=('AUTO_HOVER',time.monotonic())
        b._require_goal_altitude=Mock(); b._lock=__import__('threading').RLock()
        b.planning_preview_enabled=False; b.status_pub=Mock()
        BRIDGE.rospy.loginfo=Mock(); BRIDGE.rospy.logwarn=Mock()
        received=[]
        async def handler(reader,writer):
            raw=json.loads(await reader.readline()); assert raw.pop('auth_token')=='test-wire-token'
            cmd=parse_command(raw); received.append(cmd)
            status,detail=b._apply_operator_task(cmd)
            writer.write((json.dumps(dict(type='operator_task_ack',task_id=cmd.task_id,sequence=cmd.sequence,status=status,detail=detail))+'\n').encode())
            await writer.drain(); writer.close(); await writer.wait_closed()
        server=await asyncio.start_server(handler,'127.0.0.1',0)
        async with server:
            d=TaskDispatcher(MissionManager(control_output_enabled=True),
                OnboardBridgeClient('127.0.0.1',server.sockets[0].getsockname()[1],'test-wire-token'),
                control_output_enabled=True,operator_control_token='test-operator',live_control_confirmation='TEST',
                command_ttl_ms=500,world_frame='world',body_frame='base_link')
            for task,action in [('approach_target','APPROACH'),('retreat_target','RETREAT'),('turn_to_target','TURN'),('pass_side','PASS'),('rotate_full','ROTATE'),('land_near_target','LAND')]:
                request=TaskDispatchRequest.model_validate(dict(category='embodied',embodied_task=task,mode='live',live_confirmation='TEST',parameters=dict(target_label='chair')))
                with pytest.raises(HTTPException): await d.dispatch(request,'incorrect')
                result=await d.dispatch(request,'test-operator')
                assert result['delivery']['status']=='accepted'
                cmd=received[-1]; assert cmd.target_skill['action']==action
                body=json.loads(b.target_skill_pub.publish.call_args.args[0].data)
                assert body['task_id']==result['task_id'] and body['action']==action
                with pytest.raises(ProtocolError): b._apply_operator_task(cmd)
                b._semantic_status_callback('target',BRIDGE.String(data=json.dumps(dict(task_id=cmd.task_id,state='SUCCEEDED',detail='mock odometry confirmed'))))
                await d.ingest_onboard_status(json.loads(b.status_pub.publish.call_args.args[0].data))
                assert (await d.snapshot())['recent_tasks'][0]['runtime']['semantic_state']=='SUCCEEDED'
                assert b._active_semantic_task is None
    asyncio.run(scenario())
