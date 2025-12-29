import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths and Names
    pkg_name = 'mobile_manipulator_description'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'mobile_manipulator.urdf.xacro')
    world_file = os.path.join(get_package_share_directory(pkg_name), 'world', 'warehouse.world')

    # 2. Process URDF (Robot State Publisher)
    # This takes your XACRO and converts it to raw URDF for Gazebo
    robot_description_config = Command(['xacro ', xacro_file])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 3. Launch Gazebo with the custom world
    # This uses the standard gazebo_ros launch logic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 4. Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mobile_manipulator'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])