from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="ros"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("rover_cmd_vel_topic", default_value="/rover_base_controller/cmd_vel_unstamped"),
            DeclareLaunchArgument("rover_odom_topic", default_value="/rover_base_controller/odom"),
            DeclareLaunchArgument("rover_battery_topic", default_value="/battery_state"),
            DeclareLaunchArgument("rover_headlights_service", default_value="/power_board/set_headlights"),
            Node(
                package="silverhand_arm_ws_gateway",
                executable="gateway",
                name="silverhand_rover_ws_gateway",
                output="screen",
                arguments=[
                    "--domain",
                    "rover",
                    "--mode",
                    LaunchConfiguration("mode"),
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--rover-cmd-vel-topic",
                    LaunchConfiguration("rover_cmd_vel_topic"),
                    "--rover-odom-topic",
                    LaunchConfiguration("rover_odom_topic"),
                    "--rover-battery-topic",
                    LaunchConfiguration("rover_battery_topic"),
                    "--rover-headlights-service",
                    LaunchConfiguration("rover_headlights_service"),
                ],
            ),
        ]
    )
