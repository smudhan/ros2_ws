import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Package Paths
    pkg_description = get_package_share_directory('mobile_manipulator_description')
    pkg_moveit = get_package_share_directory('mobile_manipulator_moveit_config')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_brain = get_package_share_directory('mobile_manipulator_brain')
    
    # 2. File Paths
    pkg_description = get_package_share_directory('mobile_manipulator_description')
    map_path = os.path.join(pkg_description, 'maps', 'warehouse_map.yaml')
    params_path = os.path.join(pkg_description, 'config', 'nav2_params.yaml')
    rviz_path = os.path.join(pkg_description, 'rviz', 'rvizconfigfile_mobile_mani.rviz')

    # 3. Simulation Launch (In its own window)
    simulation = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'launch', 'mobile_manipulator_description', 'simulation.launch.py'],
        output='screen'
    )

    # 4. Controller Spawners (Staggered Delays)
    spawn_jsb = TimerAction(period=5.0, actions=[
        ExecuteProcess(cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'])
    ])
    spawn_arm = TimerAction(period=7.0, actions=[
        ExecuteProcess(cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller'])
    ])
    spawn_gripper = TimerAction(period=9.0, actions=[
        ExecuteProcess(cmd=['ros2', 'run', 'controller_manager', 'spawner', 'gripper_controller'])
    ])

    # 5. Localization & Navigation (Forced Autostart)
    nav_stack = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['xterm', '-hold', '-e', 'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py', 
                 'use_sim_time:=True', 
                 'map:=' + map_path, 
                 'params_file:=' + params_path,
                 'autostart:=True'], # Forces nodes to move to 'Active' state
            output='screen'
        )
    ])

    # 6. MoveIt2
    moveit = TimerAction(period=15.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')),
            launch_arguments={'use_sim_time': 'True'}.items()
        )
    ])

    # 7. RViz2 (Updated for stability)
    rviz = TimerAction(period=18.0, actions=[
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_path, '--ros-args', '-p', 'use_sim_time:=True'],
            output='screen'
        )
    ])

    return LaunchDescription([
        simulation,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
        nav_stack,
        moveit,
        rviz
    ])