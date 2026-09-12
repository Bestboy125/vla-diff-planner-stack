"""Requires isolated_ros_smoke.py + SSH loopback forward; never use a live bridge."""
import asyncio
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from app.mission import MissionManager
from app.onboard_bridge import OnboardBridgeClient
from app.schemas import TaskDispatchRequest
from app.task_dispatch import TaskDispatcher

async def main():
    dispatcher=TaskDispatcher(MissionManager(control_output_enabled=True),
        OnboardBridgeClient('127.0.0.1',19191,'isolated-test-token',timeout_sec=3),
        control_output_enabled=True,operator_control_token='test-only',live_control_confirmation='TEST_ONLY',
        command_ttl_ms=2000,world_frame='world',body_frame='base_link')
    for task in ('approach_target','retreat_target','turn_to_target','pass_side','rotate_full','land_near_target'):
        request=TaskDispatchRequest.model_validate(dict(category='embodied',embodied_task=task,
            mode='live',live_confirmation='TEST_ONLY',parameters=dict(target_label='chair',target_selector='left')))
        result=await dispatcher.dispatch(request,'test-only')
        assert result['delivery']['status']=='operator_locked',result['delivery']
        print(task+': host backend -> SSH loopback -> onboard parser -> operator_locked')

if __name__=='__main__': asyncio.run(main())
