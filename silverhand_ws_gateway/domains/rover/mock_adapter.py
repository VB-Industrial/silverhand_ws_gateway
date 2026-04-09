from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from dataclasses import dataclass
from typing import Any

from ...core.adapter_base import EventSink, RobotAdapter
from ...core.config import GatewayConfig
from ...core.protocol import as_dict, make_fault_state, make_message
from .protocol import GROUP_ROVER, as_drive_mode, make_rover_state, normalize_input_source


LOGGER = logging.getLogger("silverhand_ws_gateway.rover.mock")


@dataclass(slots=True)
class RoverMockState:
    drive_mode: str = "manual"
    ready: bool = True
    input_source: str = "mock_autonomy"
    estop_active: bool = False
    headlights_enabled: bool = False
    commanded_linear_m_s: float = 0.0
    commanded_angular_rad_s: float = 0.0
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    heading_deg: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    odometer_km: float = 0.0
    battery_percent: float = 86.0
    battery_voltage_v: float = 25.4
    battery_current_a: float = -1.5
    last_command_monotonic: float = 0.0


class MockRoverAdapter(RobotAdapter):
    server_name = "silverhand_ws_gateway_rover"
    groups = (GROUP_ROVER,)

    def __init__(self, config: GatewayConfig) -> None:
        self._config = config
        self._event_sink: EventSink | None = None
        self._state = RoverMockState(last_command_monotonic=time.monotonic())
        self._ticker_task: asyncio.Task[None] | None = None

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        LOGGER.info("Mock rover adapter started")
        await self._publish_full_state()
        self._ticker_task = asyncio.create_task(self._ticker_loop())

    async def stop(self) -> None:
        LOGGER.info("Mock rover adapter stopping")
        if self._ticker_task is not None:
            self._ticker_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._ticker_task
            self._ticker_task = None

    async def handle_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        payload = as_dict(message.get("payload", {}), field_name="payload")

        if message_type == "cmd_vel":
            await self._handle_cmd_vel(payload)
            return
        if message_type == "stop":
            await self._handle_stop()
            return
        if message_type == "estop":
            await self._handle_estop()
            return
        if message_type == "reset_estop":
            await self._handle_reset_estop()
            return
        if message_type == "set_drive_mode":
            await self._handle_set_drive_mode(payload)
            return
        if message_type == "set_headlights":
            await self._handle_set_headlights(payload)
            return

    async def _handle_cmd_vel(self, payload: dict[str, Any]) -> None:
        if self._state.estop_active:
            await self._emit(
                make_fault_state(
                    "rover_estop_active",
                    "Ignoring cmd_vel because E-STOP is active.",
                    severity="warning",
                    active=True,
                )
            )
            await self._publish_rover_state()
            return

        self._state.commanded_linear_m_s = float(payload.get("linear_m_s", 0.0))
        self._state.commanded_angular_rad_s = float(payload.get("angular_rad_s", 0.0))
        self._state.input_source = normalize_input_source(payload, default=self._state.input_source)
        self._state.last_command_monotonic = time.monotonic()
        await self._publish_rover_state()

    async def _handle_stop(self) -> None:
        self._state.commanded_linear_m_s = 0.0
        self._state.commanded_angular_rad_s = 0.0
        self._state.linear_m_s = 0.0
        self._state.angular_rad_s = 0.0
        self._state.last_command_monotonic = time.monotonic()
        await self._publish_odometry()
        await self._publish_rover_state()

    async def _handle_estop(self) -> None:
        self._state.estop_active = True
        await self._handle_stop()
        await self._emit(make_fault_state("rover_estop_active", "Emergency stop active.", severity="fatal", active=True))

    async def _handle_reset_estop(self) -> None:
        self._state.estop_active = False
        self._state.last_command_monotonic = time.monotonic()
        await self._emit(make_fault_state("rover_estop_active", "Emergency stop reset.", severity="info", active=False))
        await self._publish_rover_state()

    async def _handle_set_drive_mode(self, payload: dict[str, Any]) -> None:
        self._state.drive_mode = as_drive_mode(payload)
        await self._publish_rover_state()

    async def _handle_set_headlights(self, payload: dict[str, Any]) -> None:
        self._state.headlights_enabled = bool(payload.get("enabled", False))
        await self._emit(
            make_fault_state(
                "rover_headlights_changed",
                "Headlights enabled." if self._state.headlights_enabled else "Headlights disabled.",
                severity="info",
                active=False,
            )
        )
        await self._publish_rover_state()

    async def _ticker_loop(self) -> None:
        dt = max(self._config.mock_step_s, 0.02)
        while True:
            await asyncio.sleep(dt)
            self._tick(dt)
            await self._publish_odometry()
            await self._publish_battery_state()
            await self._publish_rover_state()

    def _tick(self, dt: float) -> None:
        linear_target = 0.0 if self._state.estop_active else self._state.commanded_linear_m_s
        angular_target = 0.0 if self._state.estop_active else self._state.commanded_angular_rad_s

        self._state.linear_m_s = _approach(self._state.linear_m_s, linear_target, dt * 2.2)
        self._state.angular_rad_s = _approach(self._state.angular_rad_s, angular_target, dt * 3.0)

        self._state.heading_deg = _normalize_heading(self._state.heading_deg + math.degrees(self._state.angular_rad_s) * dt)
        heading_rad = math.radians(self._state.heading_deg)
        self._state.x_m += math.sin(heading_rad) * self._state.linear_m_s * dt
        self._state.y_m += math.cos(heading_rad) * self._state.linear_m_s * dt
        self._state.roll_deg = max(-25.0, min(25.0, math.sin(time.monotonic() / 0.9) * 7.0 + self._state.angular_rad_s * 9.0))
        self._state.pitch_deg = max(-20.0, min(20.0, math.sin(time.monotonic() / 1.3) * 4.0 + self._state.linear_m_s * 3.5))
        self._state.odometer_km += abs(self._state.linear_m_s) * dt / 1000.0

        battery_drain = abs(self._state.linear_m_s) * dt * 0.018
        self._state.battery_percent = max(0.0, min(100.0, self._state.battery_percent - battery_drain))
        self._state.battery_voltage_v = 22.0 + self._state.battery_percent * 0.04

    async def _publish_full_state(self) -> None:
        await self._publish_odometry()
        await self._publish_battery_state()
        await self._publish_rover_state()

    async def _publish_odometry(self) -> None:
        await self._emit(
            make_message(
                "odometry",
                {
                    "linear_m_s": self._state.linear_m_s,
                    "angular_rad_s": self._state.angular_rad_s,
                    "heading_deg": self._state.heading_deg,
                    "odometer_km": self._state.odometer_km,
                    "x_m": self._state.x_m,
                    "y_m": self._state.y_m,
                    "roll_deg": self._state.roll_deg,
                    "pitch_deg": self._state.pitch_deg,
                },
            )
        )

    async def _publish_battery_state(self) -> None:
        await self._emit(
            make_message(
                "battery_state",
                {
                    "percent": self._state.battery_percent,
                    "voltage_v": self._state.battery_voltage_v,
                    "current_a": self._state.battery_current_a,
                },
            )
        )

    async def _publish_rover_state(self) -> None:
        await self._emit(
            make_rover_state(
                mode=self._state.drive_mode,
                ready=self._state.ready,
                control_active=not self._state.estop_active,
                headlights_enabled=self._state.headlights_enabled,
                input_source=self._state.input_source,
                signal_quality="strong",
                command_age_ms=(time.monotonic() - self._state.last_command_monotonic) * 1000.0,
            )
        )

    async def _emit(self, message: dict[str, Any]) -> None:
        if self._event_sink is None:
            return
        await self._event_sink(message)


def _normalize_heading(value: float) -> float:
    next_value = value % 360.0
    if next_value < 0.0:
        next_value += 360.0
    return next_value


def _approach(current: float, target: float, delta: float) -> float:
    if current < target:
        return min(current + delta, target)
    return max(current - delta, target)
