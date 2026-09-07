from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.192.1.2",
                description="Robot IP address",
            ),
            DeclareLaunchArgument(
                "robot_type",
                default_value="wheel_leg",
                choices=["wheel_leg", "sole_foot"],
                description="Robot type: wheel_leg or sole_foot",
            ),
            Node(
                package="limxsdk_lowlevel",
                executable="tron2_request_mode",
                name="tron2_request_mode",
                output="screen",
                parameters=[
                    {
                        "robot_ip": LaunchConfiguration("robot_ip"),
                        "robot_type": LaunchConfiguration("robot_type"),
                    }
                ],
            ),
        ]
    )
