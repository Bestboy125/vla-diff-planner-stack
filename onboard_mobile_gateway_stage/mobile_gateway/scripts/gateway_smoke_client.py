#!/usr/bin/env python3
"""Safe transport smoke test: the default safety gate must reject HOLD."""

import argparse
import asyncio
import json
import time
import uuid

import websockets


async def run(url, token):
    async with websockets.connect(url, max_size=65536) as socket:
        await socket.send(json.dumps({
            "type": "hello", "protocol_version": "1.0",
            "client_id": "onboard-smoke-test", "role": "operator_ui",
            "auth_token": token, "client_time_ms": int(time.time() * 1000),
        }))
        hello = json.loads(await asyncio.wait_for(socket.recv(), 3.0))
        if hello.get("type") != "hello_ack":
            raise RuntimeError("hello_ack not received: %r" % hello)
        status = json.loads(await asyncio.wait_for(socket.recv(), 3.0))
        if status.get("type") != "status":
            raise RuntimeError("status not received: %r" % status)

        now = int(time.time() * 1000)
        request_id = str(uuid.uuid4())
        await socket.send(json.dumps({
            "type": "command", "protocol_version": "1.0",
            "request_id": request_id, "client_id": "onboard-smoke-test",
            "issued_at_ms": now, "expires_at_ms": now + 10000,
            "action": "HOLD", "arguments": {"parameter_source": "onboard_default"},
            "source_text": "safe transport smoke test", "operator_confirmed": False,
        }))
        while True:
            response = json.loads(await asyncio.wait_for(socket.recv(), 3.0))
            if response.get("type") == "command_ack" and response.get("request_id") == request_id:
                if response.get("state") != "REJECTED_SAFETY_GATE":
                    raise RuntimeError("safety gate did not reject test command: %r" % response)
                print("MOBILE_GATEWAY_SMOKE_TEST_PASSED")
                print(json.dumps(status, ensure_ascii=False, indent=2))
                print(json.dumps(response, ensure_ascii=False, indent=2))
                return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:8765/ws/control")
    parser.add_argument("--token-file", required=True)
    args = parser.parse_args()
    with open(args.token_file, "r", encoding="utf-8") as handle:
        token = handle.read().strip()
    asyncio.get_event_loop().run_until_complete(run(args.url, token))


if __name__ == "__main__":
    main()
