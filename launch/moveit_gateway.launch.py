from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("move_group_action", default_value="/move_action"),
            Node(
                package="silverhand_arm_ws_gateway",
                executable="gateway",
                name="silverhand_arm_ws_gateway",
                output="screen",
                arguments=[
                    "--mode",
                    "moveit",
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--move-group-action",
                    LaunchConfiguration("move_group_action"),
                ],
            ),
        ]
    )
