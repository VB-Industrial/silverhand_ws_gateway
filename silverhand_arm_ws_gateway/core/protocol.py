from __future__ import annotations

import json
from typing import Any


def make_message(message_type: str, payload: dict[str, Any] | None = None, message_id: str | None = None) -> dict[str, Any]:
    message: dict[str, Any] = {"type": message_type, "payload": payload or {}}
    if message_id is not None:
        message["id"] = message_id
    return message


def dumps_message(message: dict[str, Any]) -> str:
    return json.dumps(message, ensure_ascii=True, separators=(",", ":"))


def loads_message(raw: str) -> dict[str, Any]:
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("Protocol message must be a JSON object.")
    return data


def as_dict(value: Any, *, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be an object.")
    return value


def make_hello_ack(server_name: str, groups: tuple[str, ...]) -> dict[str, Any]:
    return make_message(
        "hello_ack",
        {
            "protocol_version": 1,
            "server_name": server_name,
            "groups": list(groups),
        },
    )


def make_pong(payload: dict[str, Any]) -> dict[str, Any]:
    if "heartbeat_id" in payload:
        return make_message("pong", {"heartbeat_id": payload.get("heartbeat_id")})
    if "ts" in payload:
        return make_message("pong", {"ts": payload.get("ts")})
    return make_message("pong", {})


def make_fault_state(code: str, message: str, *, severity: str = "error", active: bool = True) -> dict[str, Any]:
    return make_message(
        "fault_state",
        {
            "code": code,
            "message": message,
            "severity": severity,
            "active": active,
        },
    )
