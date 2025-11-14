# SPDX-License-Identifier: Apache-2.0
"""
Launch both nodes; accepts CLI args to modify node behavior.
Args:
  freq_hz (double): talker publish frequency
  base_string (string): initial message prefix
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    freq_arg = DeclareLaunchArgument(
        "freq_hz", default_value="2.0",
        description="Publish frequency for talker (Hz)")
    base_arg = DeclareLaunchArgument(
        "base_string", default_value="UMD Robotics says hi!",
        description="Base string for talker")
    log_arg = DeclareLaunchArgument("log_level", default_value="info",
                                description="Logger level for both nodes")

    talker = Node(
        package="beginner_tutorials",
        executable="talker",
        name="minimal_publisher",
        output="screen",
        parameters=[{
            "publish_frequency_hz": LaunchConfiguration("freq_hz"),
            "base_string": LaunchConfiguration("base_string"),
        }],
        arguments=["--ros-args", "--log-level", "minimal_publisher:=debug"]  # or LaunchConfiguration("log_level")
    )

    listener = Node(
        package="beginner_tutorials",
        executable="listener",
        name="minimal_subscriber",
        output="screen",
        arguments=["--ros-args", "--log-level", "minimal_subscriber:=debug"]

    )

    return LaunchDescription([freq_arg, base_arg, log_arg, talker, listener])
