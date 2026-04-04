from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable
from typing import Any

from websockets.exceptions import ConnectionClosed
from websockets.server import WebSocketServerProtocol, serve

from .adapter_base import RobotAdapter
from .config import GatewayConfig
from .protocol import dumps_message, loads_message, make_fault_state, make_hello_ack, make_pong


LOGGER = logging.getLogger("silverhand_arm_ws_gateway")


def summarize_message(message: dict[str, Any]) -> str:
    message_type = message.get("type", "<unknown>")
    payload = message.get("payload", {})
    if not isinstance(payload, dict):
        return str(message_type)

    if message_type in {"set_joint_goal", "set_pose_goal"}:
        goal = payload.get("goal", payload)
        if isinstance(goal, dict):
            group_name = goal.get("group_name", "?")
            return f"{message_type} group={group_name}"
    if message_type in {"execute", "plan", "stop"}:
        group_name = payload.get("group_name") or payload.get("options", {}).get("group_name")
        return f"{message_type} group={group_name}"
    if message_type in {"hello", "ping"}:
        return str(message_type)
    return f"{message_type} payload_keys={sorted(payload.keys())}"


class GatewayServer:
    def __init__(self, config: GatewayConfig, adapter: RobotAdapter) -> None:
        self._config = config
        self._adapter = adapter
        self._clients: set[WebSocketServerProtocol] = set()
        self._server_task: asyncio.Task[None] | None = None

    async def run_forever(self) -> None:
        await self._adapter.start(self.broadcast)
        async with serve(self._handle_client, self._config.host, self._config.port, ping_interval=None):
            LOGGER.info("Gateway listening on ws://%s:%s in %s mode", self._config.host, self._config.port, self._config.mode)
            await asyncio.Future()

    async def shutdown(self) -> None:
        await self._adapter.stop()

    async def broadcast(self, message: dict[str, Any]) -> None:
        LOGGER.debug("Broadcast -> %d client(s): %s", len(self._clients), summarize_message(message))
        if not self._clients:
            return
        encoded = dumps_message(message)
        stale_clients: list[WebSocketServerProtocol] = []
        for client in tuple(self._clients):
            try:
                await client.send(encoded)
            except ConnectionClosed:
                stale_clients.append(client)
        for client in stale_clients:
            self._clients.discard(client)

    async def _handle_client(self, websocket: WebSocketServerProtocol) -> None:
        self._clients.add(websocket)
        LOGGER.info("Client connected: %s", websocket.remote_address)
        try:
            async for raw_message in websocket:
                try:
                    message = loads_message(raw_message)
                    await self._dispatch_message(websocket, message)
                except Exception as exc:  # noqa: BLE001
                    LOGGER.exception("Failed to process message")
                    await websocket.send(
                        dumps_message(
                            make_fault_state(
                                "bad_message",
                                str(exc),
                                severity="error",
                                active=True,
                            )
                        )
                    )
        finally:
            self._clients.discard(websocket)
            LOGGER.info("Client disconnected: %s", websocket.remote_address)

    async def _dispatch_message(self, websocket: WebSocketServerProtocol, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        payload = message.get("payload", {})
        LOGGER.info("Received <- %s from %s", summarize_message(message), websocket.remote_address)

        if message_type == "hello":
            await websocket.send(dumps_message(make_hello_ack("silverhand_arm_ws_gateway")))
            return
        if message_type == "ping":
            ts = payload.get("ts") if isinstance(payload, dict) else None
            await websocket.send(dumps_message(make_pong(ts)))
            return

        await self._adapter.handle_message(message)


async def run_gateway(config: GatewayConfig, adapter_factory: Callable[[GatewayConfig], RobotAdapter]) -> None:
    server = GatewayServer(config, adapter_factory(config))
    try:
        await server.run_forever()
    finally:
        with contextlib.suppress(Exception):
            await server.shutdown()
