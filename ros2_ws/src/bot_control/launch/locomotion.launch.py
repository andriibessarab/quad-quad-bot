import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "bot_control"
PARAMS_FILE_NAME = "locomotion_params.yaml"

LOCOMOTION_COMMANDER_EXE_NAME = "locomotion_commander"
GAIT_PLANNER_EXE_NAME = "limb_trajectory_generator"


def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    params_file = os.path.join(pkg_share, "config", PARAMS_FILE_NAME)

    locomotion_commander_node = Node(
        package=PACKAGE_NAME,
        executable=LOCOMOTION_COMMANDER_EXE_NAME,
        parameters=[params_file],
        output="screen",
    )

    gait_planner_node = Node(
        package=PACKAGE_NAME,
        executable=GAIT_PLANNER_EXE_NAME,
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription(
        [
            locomotion_commander_node,
            gait_planner_node,
        ]
    )
