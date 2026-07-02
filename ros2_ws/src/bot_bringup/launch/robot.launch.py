import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

BOT_CONTROL_PACKAGE_NAME = "bot_control"
BOT_SIM_PACKAGE_NAME = "bot_sim"
BOT_DESCRIPTION_PACKAGE_NAME = "bot_description"
BOT_TELEOP_PACKAGE_NAME = "bot_teleop"

HARDWARE_PLUGIN = "bot_hardware/StsSystemInterface"

LOCOMOTION_LAUNCH_FILE_NAME = "locomotion.launch.py"
TELEOP_LAUNCH_FILE_NAME = "teleop.launch.py"
SIM_LAUNCH_FILE_NAME = "sim.launch.py"
ROBOT_URDF_FILE_NAME = "robot.urdf.xacro"

RUN_MODE_ARG_NAME = "run_mode"
RUN_MODE_SIM = "sim"
RUN_MODE_REAL = "real"


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(RUN_MODE_ARG_NAME), OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    run_mode = LaunchConfiguration(RUN_MODE_ARG_NAME).perform(context).strip().lower()
    if not run_mode:
        raise RuntimeError(
            f"Missing required launch argument: {RUN_MODE_ARG_NAME}. "
            f"Use '{RUN_MODE_ARG_NAME}:={RUN_MODE_SIM}' for simulation or "
            f"'{RUN_MODE_ARG_NAME}:={RUN_MODE_REAL}' for real hardware."
        )
    if run_mode not in (RUN_MODE_SIM, RUN_MODE_REAL):
        raise RuntimeError(
            f"Invalid {RUN_MODE_ARG_NAME}='{run_mode}'. Use '{RUN_MODE_SIM}' or '{RUN_MODE_REAL}'."
        )

    # grab file paths
    bot_control_share = get_package_share_directory(BOT_CONTROL_PACKAGE_NAME)
    bot_sim_share = get_package_share_directory(BOT_SIM_PACKAGE_NAME)
    bot_description_share = get_package_share_directory(BOT_DESCRIPTION_PACKAGE_NAME)
    bot_teleop_share = get_package_share_directory(BOT_TELEOP_PACKAGE_NAME)

    # process xacro file
    xacro_file = os.path.join(bot_description_share, "urdf", ROBOT_URDF_FILE_NAME)

    # start locomotion stack from bot_control
    locomotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_control_share, "launch", LOCOMOTION_LAUNCH_FILE_NAME)
        )
    )

    # teleop: joystick -> /cmd_vel
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_teleop_share, "launch", TELEOP_LAUNCH_FILE_NAME)
        )
    )

    # publishes feedback joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # consumes joint commands and drives joints
    joint_group_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller"],
    )

    # nodes that are used in both sim and real modes
    common_nodes = [
        locomotion_launch,
        teleop_launch,
        joint_state_broadcaster_spawner,
        joint_group_position_controller_spawner,
    ]

    # Run nodes based on mode
    if run_mode == RUN_MODE_SIM:
        return _sim_nodes(xacro_file, bot_sim_share) + common_nodes
    return _real_nodes(xacro_file, bot_control_share) + common_nodes


def _sim_nodes(xacro_file, bot_sim_share):
    # process xacro file
    robot_desc_content = Command(["xacro ", xacro_file, " sim:=true"])

    # start Gazebo from bot_sim.
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bot_sim_share, "launch", SIM_LAUNCH_FILE_NAME))
    )

    # publishes kinematic transform tree
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc_content, "use_sim_time": True}],
    )

    return [sim_launch, robot_state_publisher_node]


def _real_nodes(xacro_file, bot_control_share):
    robot_desc = Command(["xacro ", xacro_file, " sim:=false hardware_plugin:=", HARDWARE_PLUGIN])

    # publishes kinematic transform tree
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )

    # Standalone controller manager — takes the place of Gazebo's embedded one.
    # Needs the robot description (to load the hardware plugin) and the
    # controllers config (to know which controllers exist and their joint lists).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            os.path.join(bot_control_share, "config", "controllers.yaml"),
        ],
    )

    return [ros2_control_node, robot_state_publisher_node]
