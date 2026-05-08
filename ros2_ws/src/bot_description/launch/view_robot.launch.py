import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# constants
BOT_DESCRIPTION_PKG_NAME = "bot_description"
MODEL_ARG_NAME = "model"
RVIZCONFIG_ARG_NAME = "rvizconfig"
DEFAULT_MODEL_FILE_NAME = "robot.urdf.xacro"
DEFAULT_RVIZCONFIG_FILE_NAME = "robot.rviz"


def generate_launch_description():
    # file paths
    pkg_share = get_package_share_directory(BOT_DESCRIPTION_PKG_NAME)

    # define default  model and rviz config
    default_model = os.path.join(pkg_share, "urdf", DEFAULT_MODEL_FILE_NAME)
    default_rvizconfig = os.path.join(pkg_share, "rviz", DEFAULT_RVIZCONFIG_FILE_NAME)

    # make xacro model and rviz config overridable form terminal
    model_arg = DeclareLaunchArgument(
        MODEL_ARG_NAME,
        default_value=default_model,
        description="Absolute path to URDF/Xacro model file",
    )
    rviz_arg = DeclareLaunchArgument(
        RVIZCONFIG_ARG_NAME,
        default_value=default_rvizconfig,
        description="Absolute path to RViz config file",
    )

    # Process the Xacro file
    robot_description = Command(["xacro ", LaunchConfiguration(MODEL_ARG_NAME)])

    # define state publisher & rviz nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration(RVIZCONFIG_ARG_NAME)],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
