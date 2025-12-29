import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get package directories
    pkg_description = get_package_share_directory('mobile_manipulator_description')
    pkg_brain = get_package_share_directory('mobile_manipulator_brain')

    # 2. Path to your RViz config (Dynamic path for portability)
    rviz_config_path = os.path.join(pkg_brain, 'rviz', 'teleoprvizconfig.rviz')

    # 3. Include the Gazebo Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'simulation.launch.py')
        )
    )

    # 4. Controller Spawners (Wrapped in TimerAction to prevent race conditions)
    spawn_jsb = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'],
            output='screen'
        )]
    )

    spawn_arm = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller'],
            output='screen'
        )]
    )

    spawn_gripper = TimerAction(
        period=9.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', 'gripper_controller'],
            output='screen'
        )]
    )

    # 5. Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
        rviz_node,
    ])