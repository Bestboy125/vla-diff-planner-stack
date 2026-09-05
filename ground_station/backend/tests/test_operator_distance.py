from unittest.mock import AsyncMock

import pytest
from fastapi.testclient import TestClient
from app import main


@pytest.mark.parametrize('distance,expected', [(0.05, 200), (1.5, 200), (2.0, 200), (2.001, 422), (0.049, 422)])
def test_distance_boundary_without_delivery(monkeypatch, distance, expected):
    send = AsyncMock(side_effect=AssertionError('No onboard delivery allowed'))
    monkeypatch.setattr(main.task_dispatcher.onboard_bridge, 'send', send)
    response = TestClient(main.app).post('/api/tasks/dispatch', json={
        'category': 'atomic', 'atomic_task': 'move_forward', 'mode': 'dry_run',
        'parameters': {'distance_m': distance},
    })
    assert response.status_code == expected
    if expected == 200:
        assert response.json()['command']['magnitude'] == distance
        assert response.json()['delivery']['status'] == 'safety_locked'
    send.assert_not_called()


def test_catalog_matches_schema():
    result = TestClient(main.app).get('/api/tasks/catalog').json()
    assert result['limits']['distance_m'] == [0.05, 2.0]
