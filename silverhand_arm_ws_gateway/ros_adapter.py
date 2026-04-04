from __future__ import annotations

from .adapter_base import EventSink, RobotAdapter
from .config import GatewayConfig
from .protocol import make_fault_state


class RosRobotAdapter(RobotAdapter):
    def __init__(self, config: GatewayConfig) -> None:
        self._config = config
        self._event_sink: EventSink | None = None

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        await self._emit(
            make_fault_state(
                "ros_adapter_stub",
                "ROS adapter scaffold is ready, but topic/action wiring is not implemented yet.",
                severity="info",
                active=False,
            )
        )

    async def stop(self) -> None:
        return

    async def handle_message(self, message: dict[str, object]) -> None:
        await self._emit(
            make_fault_state(
                "ros_adapter_stub",
                f"ROS adapter received '{message.get('type')}' but wiring is not implemented yet.",
                severity="warning",
                active=True,
            )
        )

    async def _emit(self, message: dict[str, object]) -> None:
        if self._event_sink is None:
            return
        await self._event_sink(message)
