import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

HARDWARE_PLUGIN = "bot_hardware/StsSystemInterface"


def generate_launch_description():
    bot_description_share = get_package_share_directory("bot_description")
    bot_control_share = get_package_share_directory("bot_control")

    xacro_file = os.path.join(bot_description_share, "urdf", "robot.urdf.xacro")
    robot_desc = Command(["xacro ", xacro_file, " sim:=false hardware_plugin:=", HARDWARE_PLUGIN])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            os.path.join(bot_control_share, "config", "controllers.yaml"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller"],
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        joint_group_position_controller,
    ])
