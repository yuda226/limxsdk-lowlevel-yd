"""
Combined launch file for TRON2 mode management and velocity control.

This launch file starts:
1. tron2_request_mode: Handles walking state initialization (stand -> walk)
2. tron2_cmd_vel_translator: Translates ROS2 Twist messages to robot velocity commands

Usage:
    ros2 launch limxsdk_lowlevel tron2_complete_setup.launch.py [robot_type:=wheel_leg|sole_foot]

Note: Call the services in this order before sending cmd_vel commands:
    1. ros2 service call /tron2_stand_mode std_srvs/Trigger
    2. ros2 service call /tron2_walk_mode std_srvs/Trigger
    3. Then publish to /cmd_vel topic
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_nodes(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    
    # Velocity parameters differ between wheel-leg and sole-foot modes
    if robot_type == "wheel_leg":
        # Wheel-leg mode uses actual velocities (m/s, rad/s)
        max_linear_x = 2.0   # m/s
        max_linear_y = 0.0   # not supported in wheel-leg mode
        max_angular_z = 1.0  # rad/s
    else:
        # Sole-foot mode uses normalized ratios [-1, 1]
        max_linear_x = 1.0
        max_linear_y = 1.0
        max_angular_z = 1.0

    return [
        # Mode management node (must run first)
        Node(
            package="limxsdk_lowlevel",
            executable="tron2_request_mode",
            name="tron2_request_mode",
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "robot_type": robot_type,
                }
            ],
        ),
        # Velocity translator node
        Node(
            package="limxsdk_lowlevel",
            executable="tron2_cmd_vel_translator",
            name="tron2_cmd_vel_translator",
            output="screen",
            parameters=[
                {
                    "enable_sdk_output": True,
                    "enable_relay_output": False,
                    "robot_ip": robot_ip,
                    "cmd_vel_topic_in": "/cmd_vel",
                    "cmd_vel_topic_out": "/tron2/cmd_vel",
                    "max_linear_x": max_linear_x,
                    "max_linear_y": max_linear_y,
                    "max_angular_z": max_angular_z,
                    "deadman_timeout_s": 0.5,
                    "sdk_json_template": '{"accid":"nav_bridge","title":"request_twist","timestamp":{timestamp_ms},"guid":"{guid}","data":{"x":{linear_x},"y":{linear_y},"z":{angular_z}}}',
                }
            ],
        ),
    ]


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
                description="Robot type: wheel_leg (WF_TRON2A) or sole_foot (SF_TRON2A)",
            ),
            OpaqueFunction(function=_create_nodes),
        ]
    )
