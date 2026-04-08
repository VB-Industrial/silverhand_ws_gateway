from __future__ import annotations

from collections.abc import Callable

from .core.adapter_base import RobotAdapter
from .core.config import GatewayConfig
from .domains.arm import MockRobotAdapter, MoveItRobotAdapter, RosRobotAdapter
from .domains.rover import MockRoverAdapter, RoverRosAdapter


AdapterFactory = Callable[[GatewayConfig], RobotAdapter]


def resolve_adapter_factory(config: GatewayConfig) -> AdapterFactory:
    if config.domain == "arm":
        if config.mode == "mock":
            return MockRobotAdapter
        if config.mode == "ros":
            return RosRobotAdapter
        if config.mode == "moveit":
            return MoveItRobotAdapter
        raise ValueError(f"Unsupported arm mode: {config.mode!r}")

    if config.domain == "rover":
        if config.mode == "mock":
            return MockRoverAdapter
        if config.mode == "ros":
            return RoverRosAdapter
        raise ValueError(f"Unsupported rover mode: {config.mode!r}")

    raise ValueError(f"Unsupported domain: {config.domain!r}")
