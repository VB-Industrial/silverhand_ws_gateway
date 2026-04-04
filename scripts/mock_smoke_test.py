#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
from typing import Any

import websockets


async def recv_json(ws: websockets.ClientConnection) -> dict[str, Any]:
    raw = await ws.recv()
    print(raw)
    return json.loads(raw)


async def main() -> None:
    parser = argparse.ArgumentParser(description="Smoke-test client for silverhand_arm_ws_gateway mock mode")
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    args = parser.parse_args()

    async with websockets.connect(args.url) as ws:
        await ws.send(json.dumps({"type": "hello", "payload": {"protocol_version": 1, "client_name": "smoke-test"}}))
        await recv_json(ws)

        await ws.send(json.dumps({"type": "ping", "payload": {"ts": 12345}}))
        await recv_json(ws)

        await ws.send(
            json.dumps(
                {
                    "type": "set_joint_goal",
                    "payload": {
                        "command_id": "smoke-goal",
                        "goal": {
                            "group_name": "arm",
                            "joint_names": [
                                "arm_joint_1",
                                "arm_joint_2",
                                "arm_joint_3",
                                "arm_joint_4",
                                "arm_joint_5",
                                "arm_joint_6",
                            ],
                            "positions_rad": [0.0, 1.8, -0.6, 0.0, 1.57, 0.0],
                        },
                    },
                }
            )
        )
        await recv_json(ws)

        await ws.send(json.dumps({"type": "execute", "payload": {"command_id": "smoke-exec", "group_name": "arm"}}))
        for _ in range(8):
            await recv_json(ws)


if __name__ == "__main__":
    asyncio.run(main())
