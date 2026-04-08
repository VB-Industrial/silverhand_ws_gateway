from __future__ import annotations

from typing import Any

from ...core.protocol import as_dict, dumps_message, loads_message, make_fault_state, make_hello_ack, make_message, make_pong


GROUP_ARM = "arm"
GROUP_GRIPPER = "gripper"
KNOWN_GROUPS = (GROUP_ARM, GROUP_GRIPPER)


def as_group_name(payload: dict[str, Any]) -> str:
    group_name = payload.get("group_name")
    if group_name not in KNOWN_GROUPS:
        raise ValueError(f"Unsupported group_name: {group_name!r}")
    return str(group_name)
