import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    # Set the Gazebo version to harmonic (8)
    if 'GZ_VERSION' not in os.environ:
        os.environ['GZ_VERSION'] = '8'

    # Grab file paths
    pkg_bot_description = get_package_share_directory("bot_description")
    resource_path = str(Path(pkg_bot_description).parent)  # where gazebo looks for package names
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Process the Xacro file
    xacro_file = os.path.join(pkg_bot_description, "urdf", "quad_quad_limb.urdf.xacro")
    robot_desc_content = Command(['xacro ', xacro_file])

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes/models
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path,
    )

    # Start Gazebo Harmonic with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[('gz_args', '-r empty.sdf')],
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc_content,
            "use_sim_time": True
        }],
    )

    # Spawn robot into Gazebo
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "quad_limb",
            "-z", "0.1"
        ],
        output="screen",
    )

    # Spawn joint state broadcaster
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Spawn trajectory controller
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    # Bridge the clock so ROS 2 and Gazebo stay in sync
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    return LaunchDescription([
        set_gz_path,
        gazebo,
        rsp_node,
        spawn_node,
        load_jsb,
        load_jtc,
        clock_bridge
    ])