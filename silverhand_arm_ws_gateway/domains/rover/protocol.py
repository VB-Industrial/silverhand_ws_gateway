from __future__ import annotations

from typing import Any

from ...core.protocol import as_dict, make_message


GROUP_ROVER = "rover"
KNOWN_GROUPS = (GROUP_ROVER,)
KNOWN_DRIVE_MODES = ("manual", "crab", "precision", "docking")
KNOWN_INPUT_SOURCES = ("keyboard_mouse", "joystick", "mock_autonomy")


def as_group_name(payload: dict[str, Any]) -> str:
    group_name = payload.get("group_name", GROUP_ROVER)
    if group_name not in KNOWN_GROUPS:
        raise ValueError(f"Unsupported group_name: {group_name!r}")
    return str(group_name)


def as_drive_mode(payload: dict[str, Any]) -> str:
    mode = payload.get("mode")
    if mode not in KNOWN_DRIVE_MODES:
        raise ValueError(f"Unsupported drive mode: {mode!r}")
    return str(mode)


def normalize_input_source(payload: dict[str, Any], *, default: str = "keyboard_mouse") -> str:
    source = payload.get("source", default)
    if source not in KNOWN_INPUT_SOURCES:
        return default
    return str(source)


def make_rover_state(
    *,
    mode: str,
    ready: bool,
    control_active: bool,
    headlights_enabled: bool,
    input_source: str,
    signal_quality: str,
    command_age_ms: float,
) -> dict[str, Any]:
    return make_message(
        "rover_state",
        {
            "mode": mode,
            "ready": ready,
            "control_active": control_active,
            "headlights_enabled": headlights_enabled,
            "input_source": input_source,
            "signal_quality": signal_quality,
            "command_age_ms": int(max(command_age_ms, 0.0)),
        },
    )
