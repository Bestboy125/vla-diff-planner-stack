import asyncio
from unittest.mock import AsyncMock

import pytest

from app.schemas import OnboardObservation, PolicyName, MissionState
from test_image_odom_mode import payload, pipeline


def test_api_diagnostic_route_does_not_dispatch(payload, monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    monkeypatch.setattr('app.observation_pipeline.time.time', lambda: 1000)
    p = setup_pipeline(payload)
    monkeypatch.setattr(main, 'observation_pipeline', p)
    result = TestClient(main.app).post('/api/inference/latest-observation',
                                     json={'instruction': 'Move forward', 'policy': 'openvla'})
    assert result.status_code == 200
    assert result.json()['motion_command_sent'] is False
    p.onboard_bridge.send.assert_not_called()


@pytest.mark.parametrize('height,code', [(0.8, 200), (1.0, 422)])
def test_takeoff_height_contract_without_delivery(monkeypatch, height, code):
    from fastapi.testclient import TestClient
    from app import main
    send = AsyncMock(side_effect=AssertionError('must not send'))
    monkeypatch.setattr(main.task_dispatcher.onboard_bridge, 'send', send)
    result = TestClient(main.app).post('/api/tasks/dispatch', json={
        'category': 'atomic', 'atomic_task': 'takeoff', 'mode': 'dry_run',
        'parameters': {'takeoff_height_m': height},
    })
    assert result.status_code == code
    send.assert_not_called()


def setup_pipeline(payload):
    p = pipeline()
    p._latest = OnboardObservation(**payload)
    p.mission_manager = AsyncMock()
    p.mission_manager.current.return_value = None
    p.model_gateway = AsyncMock()
    p.model_gateway.predict_openvla.return_value = {'action_local_delta': [[0.1, 0, 0, 0]]}
    p.onboard_bridge = AsyncMock()
    return p


def test_diagnostic_returns_action_without_mission_or_send(payload, monkeypatch):
    monkeypatch.setattr('app.observation_pipeline.time.time', lambda: 1000)
    p = setup_pipeline(payload)
    result = asyncio.run(p.infer_latest_no_motion('Move forward', PolicyName.OPENVLA))
    assert result['motion_command_sent'] is False
    assert result['flight_execution_validated'] is False
    assert result['observation_sequence'] == 1
    assert result['target_world'] == [[0.1, 0, 0, 0]]
    p.onboard_bridge.send.assert_not_called()
    p.mission_manager.create.assert_not_called()
    p.mission_manager.start.assert_not_called()
    assert p._last_result is None
    assert p._diagnostic_busy is False


@pytest.mark.parametrize('case', ['stale', 'missing', 'running', 'busy', 'nan', 'failed'])
def test_failures_never_send_and_release_lock(payload, monkeypatch, case):
    monkeypatch.setattr('app.observation_pipeline.time.time', lambda: 1000)
    p = setup_pipeline(payload)
    if case == 'stale':
        p._latest.capture_unix_ms -= 751
    elif case == 'missing':
        p._latest = None
    elif case == 'running':
        p.mission_manager.current.return_value = type('Mission', (), {'state': MissionState.RUNNING})()
    elif case == 'busy':
        p._diagnostic_busy = True
    elif case == 'nan':
        p.model_gateway.predict_openvla.return_value = {'action_local_delta': [[float('nan'), 0, 0, 0]]}
    else:
        p.model_gateway.predict_openvla.side_effect = RuntimeError('model offline')
    with pytest.raises((ValueError, RuntimeError)):
        asyncio.run(p.infer_latest_no_motion('Move forward', PolicyName.OPENVLA))
    p.onboard_bridge.send.assert_not_called()
    if case != 'busy':
        assert p._diagnostic_busy is False
    if case in {'stale', 'missing', 'running', 'busy'}:
        p.model_gateway.predict_openvla.assert_not_called()
