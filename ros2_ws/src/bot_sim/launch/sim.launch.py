import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Grab file paths
    pkg_bot_description = get_package_share_directory('bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Process the URDF file
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'quad_quad_limb.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = {'robot_description': f.read()}

    # Set GZ_SIM_RESOURCE_PATH for Gazebo to resolve model:// URIs using
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_bot_description),
    )

    #  Start Gazebo Harmonic with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc, {'use_sim_time': True}]
    )

    # Spawn robot into Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'quad_limb', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([set_gz_path, gazebo, rsp_node, spawn_node])
