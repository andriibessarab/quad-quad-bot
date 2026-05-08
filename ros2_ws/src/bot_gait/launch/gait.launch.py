import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

BOT_GAIT_PKG_NAME = "bot_gait"
LIMB_TRAJECTORY_GENERATOR_EXE_NAME = "limb_trajectory_generator"
GAIT_PARAMS_FILE_NAME = "gait_params.yaml"


def generate_launch_description():
    pkg_bot_command = get_package_share_directory(BOT_GAIT_PKG_NAME)
    config_file = os.path.join(pkg_bot_command, "config", GAIT_PARAMS_FILE_NAME)

    limb_trajectory_generator_node = Node(
        package=BOT_GAIT_PKG_NAME,
        executable=LIMB_TRAJECTORY_GENERATOR_EXE_NAME,
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([limb_trajectory_generator_node])
