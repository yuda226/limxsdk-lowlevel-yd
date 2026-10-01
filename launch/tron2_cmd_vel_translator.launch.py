from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_node(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)

    # Default settings from user-provided arguments
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    cmd_vel_topic_in = LaunchConfiguration("cmd_vel_topic_in").perform(context)
    cmd_vel_topic_out = LaunchConfiguration("cmd_vel_topic_out").perform(context)

    max_linear_x = float(LaunchConfiguration("max_linear_x").perform(context))
    max_linear_y = float(LaunchConfiguration("max_linear_y").perform(context))
    max_angular_z = float(LaunchConfiguration("max_angular_z").perform(context))
    deadman_timeout_s = float(LaunchConfiguration("deadman_timeout_s").perform(context))
    sdk_json_template = LaunchConfiguration("sdk_json_template").perform(context)

    # Mode-specific parameter mapping.
    # mode values:
    # - onboard
    # - offboard_relay
    # - onboard_bridge_consumer
    # - offboard_direct
    if mode in ("onboard", "offboard_direct", "onboard_bridge_consumer"):
        enable_sdk_output = True
    else:
        enable_sdk_output = False

    if mode == "offboard_relay":
        enable_relay_output = True
    else:
        enable_relay_output = False

    if mode == "onboard_bridge_consumer":
        cmd_vel_topic_in = "/tron2/cmd_vel"
    elif mode == "offboard_relay":
        cmd_vel_topic_in = "/cmd_vel"
        cmd_vel_topic_out = "/tron2/cmd_vel"

    return [
        Node(
            package="limxsdk_lowlevel",
            executable="tron2_cmd_vel_translator",
            name="tron2_cmd_vel_translator",
            output="screen",
            parameters=[
                {
                    "enable_sdk_output": enable_sdk_output,
                    "enable_relay_output": enable_relay_output,
                    "robot_ip": robot_ip,
                    "cmd_vel_topic_in": cmd_vel_topic_in,
                    "cmd_vel_topic_out": cmd_vel_topic_out,
                    "max_linear_x": max_linear_x,
                    "max_linear_y": max_linear_y,
                    "max_angular_z": max_angular_z,
                    "deadman_timeout_s": deadman_timeout_s,
                    "sdk_json_template": sdk_json_template,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="onboard",
                choices=[
                    "onboard",
                    "offboard_relay",
                    "onboard_bridge_consumer",
                    "offboard_direct",
                ],
                description=(
                    "Deployment mode: onboard | offboard_relay | "
                    "onboard_bridge_consumer | offboard_direct"
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.192.1.2",
                description="Robot IP used when SDK output is enabled",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic_in",
                default_value="/cmd_vel",
                description="Input Twist topic",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic_out",
                default_value="/tron2/cmd_vel",
                description="Output Twist topic used by relay mode",
            ),
            DeclareLaunchArgument(
                "max_linear_x",
                default_value="0.6",
                description="Clamp for linear.x in m/s",
            ),
            DeclareLaunchArgument(
                "max_linear_y",
                default_value="0.4",
                description="Clamp for linear.y in m/s",
            ),
            DeclareLaunchArgument(
                "max_angular_z",
                default_value="1.2",
                description="Clamp for angular.z in rad/s",
            ),
            DeclareLaunchArgument(
                "deadman_timeout_s",
                default_value="0.5",
                description="Timeout before auto-stop command",
            ),
            DeclareLaunchArgument(
                "sdk_json_template",
                default_value='{"accid":"nav_bridge","title":"request_twist","timestamp":{timestamp_ms},"guid":"{guid}","data":{"linear_x":{linear_x},"linear_y":{linear_y},"angular_z":{angular_z}}}',
                description="JSON template for SDK publishJsonMessage payload",
            ),
            OpaqueFunction(function=_create_node),
        ]
    )
