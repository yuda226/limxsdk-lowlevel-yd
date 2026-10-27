from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "robot_ip", default_value="10.192.1.2", description="Robot IP address"
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="wheel_leg",
            choices=["wheel_leg", "sole_foot"],
            description="Robot configuration",
        ),
        DeclareLaunchArgument(
            "accid",
            default_value="",
            description="Robot serial number; empty waits for robot notifications",
        ),
        DeclareLaunchArgument(
            "offline_mode",
            default_value="false",
            choices=["true", "false"],
            description="Skip robot connection and reject commands as dry runs",
        ),
        DeclareLaunchArgument(
            "ack_timeout_s", default_value="3.0", description="Acknowledgment timeout in seconds"
        ),
        DeclareLaunchArgument(
            "stale_timeout_s", default_value="5.0", description="Receive freshness timeout in seconds"
        ),
        DeclareLaunchArgument(
            "ack_result_pointer",
            default_value="/data/result",
            description="JSON pointer to the acknowledgment result string",
        ),
        DeclareLaunchArgument(
            "ack_success_value",
            default_value="success",
            description="Acknowledgment result string indicating acceptance",
        ),
    ]
    parameter_types = {
        "robot_ip": str,
        "robot_type": str,
        "accid": str,
        "offline_mode": bool,
        "ack_timeout_s": float,
        "stale_timeout_s": float,
        "ack_result_pointer": str,
        "ack_success_value": str,
    }
    control_node = Node(
        package="limxsdk_lowlevel",
        executable="tron2_control_mode",
        name="tron2_control_mode",
        output="screen",
        parameters=[
            {
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, value_type in parameter_types.items()
            }
        ],
    )
    return LaunchDescription(arguments + [control_node])