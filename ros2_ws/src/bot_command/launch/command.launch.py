import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

BOT_COMMAND_PKG_NAME = 'bot_command'
LIMB_COMMANDER_EXE_NAME = 'limb_commander'
COMMANDER_PARAMS_FILE_NAME = 'commander_params.yaml'

def generate_launch_description():
    pkg_bot_command = get_package_share_directory(BOT_COMMAND_PKG_NAME)
    config_file = os.path.join(pkg_bot_command, "config", COMMANDER_PARAMS_FILE_NAME)

    limb_command_node = Node(
        package=BOT_COMMAND_PKG_NAME,
        executable=LIMB_COMMANDER_EXE_NAME,
        parameters=[config_file],
        output="screen"
    )

    return LaunchDescription([limb_command_node])
