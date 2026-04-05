from dataclasses import dataclass


@dataclass(slots=True)
class GatewayConfig:
    mode: str = "mock"
    host: str = "0.0.0.0"
    port: int = 8765
    move_group_action_name: str = "/move_action"
    heartbeat_interval_s: float = 3.0
    mock_step_s: float = 0.05
    mock_steps_per_execute: int = 20
    mock_max_joint_velocity_rad_s: float = 1.2
