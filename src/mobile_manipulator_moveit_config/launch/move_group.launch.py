import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # This builder now points to the specific SRDF we moved
    moveit_config = (
        MoveItConfigsBuilder("mobile_manipulator", package_name="mobile_manipulator_moveit_config")
        .robot_description(file_path="config/mobile_manipulator.urdf") # It will try to find this, but we'll override
        .robot_description_semantic(file_path="config/mobile_manipulator.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start the Move Group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([run_move_group_node])