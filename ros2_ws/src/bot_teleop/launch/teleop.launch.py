import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "bot_teleop"
XBOX_CONTROLLER_CONFIG_FILE_NAME = "xbox_controller.yaml"


def generate_launch_description():
    xbox_config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", XBOX_CONTROLLER_CONFIG_FILE_NAME
    )

    # reads raw controller input from the OS and publishes sensor_msgs/Joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    # converts Joy messages to geometry_msgs/Twist and publishes to /cmd_vel
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[xbox_config_file],
        output="screen",
    )

    return LaunchDescription([joy_node, teleop_node])
