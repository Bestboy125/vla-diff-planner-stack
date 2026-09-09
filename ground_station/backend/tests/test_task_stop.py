from unittest.mock import AsyncMock

from fastapi.testclient import TestClient
from app import main


def test_stop_dry_run_never_sends(monkeypatch):
    sender = AsyncMock(side_effect=AssertionError('real control forbidden'))
    monkeypatch.setattr(main.task_dispatcher.onboard_bridge, 'send', sender)
    result = TestClient(main.app).post('/api/tasks/stop', json={})
    assert result.status_code == 200
    assert result.json()['command']['command'] == 'HOLD'
    sender.assert_not_called()


def test_stop_live_still_requires_authorization(monkeypatch):
    monkeypatch.setattr(main.task_dispatcher, 'control_output_enabled', True)
    monkeypatch.setattr(main.task_dispatcher, 'operator_control_token', 'test-token')
    sender = AsyncMock(side_effect=AssertionError('real control forbidden'))
    monkeypatch.setattr(main.task_dispatcher.onboard_bridge, 'send', sender)
    result = TestClient(main.app).post('/api/tasks/stop', json={'mode': 'live'})
    assert result.status_code == 401
    sender.assert_not_called()


def test_authorized_stop_is_only_hold(monkeypatch):
    monkeypatch.setattr(main.task_dispatcher, 'control_output_enabled', True)
    monkeypatch.setattr(main.task_dispatcher, 'operator_control_token', 'test-token')
    monkeypatch.setattr(main.task_dispatcher, 'live_control_confirmation', 'test-confirm')
    sender = AsyncMock(return_value={'status': 'stop_requested'})
    monkeypatch.setattr(main.task_dispatcher.onboard_bridge, 'send', sender)
    result = TestClient(main.app).post('/api/tasks/stop', headers={'X-Operator-Token': 'test-token'},
        json={'mode': 'live', 'live_confirmation': 'test-confirm'})
    assert result.status_code == 200
    assert sender.call_args.args[0]['command'] == 'HOLD'
    assert sender.call_args.args[0]['magnitude'] == 0
