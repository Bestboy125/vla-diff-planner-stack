"""Exercise the packaged HTTP/static/WebSocket service on loopback, always locked."""
import asyncio
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen


def main():
    root = Path(__file__).resolve().parent
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    env.update(CONTROL_OUTPUT_ENABLED='false', ONBOARD_BRIDGE_HOST='127.0.0.1',
               ONBOARD_BRIDGE_PORT='1', OPENVLA_URL='http://127.0.0.1:1',
               PI05_HOST='127.0.0.1', PI05_PORT='1',
               ONBOARD_BRIDGE_TOKEN='test-only', OPERATOR_CONTROL_TOKEN='test-only',
               ONBOARD_OBSERVATION_TOKEN='test-only', EXPECTED_CALIBRATION_ID='test-only',
               VLA_OBSERVATION_MODE='image_odom')
    # Ignore machine-level UI parameters that could change imports or test timing.
    env.update(GROUND_STATION_PORT=str(port), STATUS_UPDATE_HZ='1')
    base = f'http://127.0.0.1:{port}'
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen(
            [sys.executable, '-m', 'uvicorn', 'app.main:app', '--host', '127.0.0.1', '--port', str(port)],
            cwd=root/'ground_station'/'backend', env=env, stdout=log, stderr=log,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        try:
            for _ in range(80):
                if process.poll() is not None:
                    raise RuntimeError('Packaged backend exited during smoke test')
                try:
                    with urlopen(base+'/api/missions/current', timeout=1) as response:
                        assert json.load(response)['mission'] is None
                    break
                except OSError:
                    time.sleep(.25)
            else:
                raise RuntimeError('Packaged backend did not become ready')
            with urlopen(base, timeout=3) as response:
                html = response.read().decode('utf-8')
            assets = re.findall(r'(?:src|href)="(/assets/[^\"]+)"', html)
            assert assets, 'No built frontend assets referenced'
            for asset in assets:
                with urlopen(base+asset, timeout=3) as response:
                    assert response.status == 200 and response.read(), asset
            with urlopen(base+'/api/health', timeout=5) as response:
                health = json.load(response)
                assert health['control_output_enabled'] is False and health['safety_lock'] is True
            async def websocket_check():
                from websockets.asyncio.client import connect
                async with connect(f'ws://127.0.0.1:{port}/ws/status', open_timeout=5) as ws:
                    status = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    assert status['safety_lock'] is True
            asyncio.run(websocket_check())
            print('PASS: built frontend, assets, HTTP API, WebSocket; control locked; no flight requests.')
        except Exception:
            log.seek(0)
            print(log.read().decode('utf-8', errors='replace'), file=sys.stderr)
            raise
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
