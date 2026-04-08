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


async def recv_until(ws: websockets.ClientConnection, expected_type: str, *, max_messages: int = 8) -> dict[str, Any]:
    for _ in range(max_messages):
        message = await recv_json(ws)
        if message.get("type") == expected_type:
            return message
    raise RuntimeError(f"Did not receive message type {expected_type!r} within {max_messages} messages")


async def recv_until_predicate(
    ws: websockets.ClientConnection,
    predicate,
    *,
    description: str,
    max_messages: int = 8,
) -> dict[str, Any]:
    for _ in range(max_messages):
        message = await recv_json(ws)
        if predicate(message):
            return message
    raise RuntimeError(f"Did not receive {description} within {max_messages} messages")


async def main() -> None:
    parser = argparse.ArgumentParser(description="Smoke-test client for silverhand_arm_ws_gateway mock mode")
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--domain", choices=("arm", "rover"), default="arm")
    args = parser.parse_args()

    async with websockets.connect(args.url) as ws:
        hello_payload: dict[str, Any] = {"protocol_version": 1, "client_name": "smoke-test"}
        if args.domain == "arm":
            hello_payload["requested_groups"] = ["arm", "gripper"]
        else:
            hello_payload["requested_groups"] = ["rover"]
        await ws.send(json.dumps({"type": "hello", "payload": hello_payload}))
        await recv_json(ws)

        await ws.send(json.dumps({"type": "ping", "payload": {"heartbeat_id": "smoke-hb"}}))
        await recv_json(ws)

        if args.domain == "rover":
            await ws.send(
                json.dumps(
                    {
                        "type": "cmd_vel",
                        "payload": {
                            "command_id": "rover-smoke",
                            "frame_id": "base_link",
                            "linear_m_s": 0.2,
                            "angular_rad_s": 0.1,
                            "source": "mock_autonomy",
                            "turbo": False,
                        },
                    }
                )
            )
            await recv_until(ws, "odometry")

            await ws.send(
                json.dumps(
                    {
                        "type": "set_headlights",
                        "payload": {
                            "command_id": "rover-headlights",
                            "enabled": True,
                        },
                    }
                )
            )
            await recv_until_predicate(
                ws,
                lambda message: message.get("type") == "rover_state"
                and message.get("payload", {}).get("headlights_enabled") is True,
                description="rover_state with headlights_enabled=true",
            )
            return

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
