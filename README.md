Mobile Manipulator Coordination System
This workspace integrates a mobile base (TurtleBot3) with an industrial manipulator (OpenXManipulator) to perform autonomous navigation and pick-and-place tasks within a warehouse environment.

🛠️ Prerequisites & Installation
Ensure you are using ROS2 Humble on Ubuntu 22.04.

Install Dependencies:


sudo apt update
sudo apt install xterm ros-humble-nav2-bringup ros-humble-moveit-ros-move-group ros-humble-joint-trajectory-controller
rosdep install --from-paths src --ignore-src -r -y 
Build the Workspace:



cd ~/ros2_ws
colcon build 
source install/setup.bash 
🕹️ Step 1: Manual Control & Teleoperation Verification
This step verifies that the ros2_control hardware interfaces for the base, arm, and gripper are correctly configured.


Launch the Integrated Environment: In Terminal 1, run the master teleop launch file:



ros2 launch mobile_manipulator_brain teleop_all.launch.py

This will start Gazebo, RViz, and all necessary controller spawners.

Run Teleop Manager: In Terminal 2, run the custom C++ teleop node:



ros2 run mobile_manipulator_brain teleop_manager
Controls: Use W/A/S/D for the base, 1-8 for arm joints, and 0 to toggle the gripper. Press X for an Emergency Stop.

Verification: Confirm the robot moves in Gazebo and RViz. Close both terminals before proceeding to Step 2.

🚛 Step 2: Full Stack Verification (Nav2 & MoveIt2)
This step demonstrates the integration of the navigation and manipulation stacks. Run each command in a new terminal:
+2

Start Simulation:



ros2 launch mobile_manipulator_description simulation.launch.py
Spawn Controllers:



ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
Launch Localization:


ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description/maps/warehouse_map.yaml
Launch Navigation:


ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description/config/nav2_params.yaml
Open RViz: (Use this command if the navigation launch doesn't open it automatically)


rviz2 -d $(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description/rviz/rvizconfigfile_mobile_mani.rviz
Launch MoveIt2:


ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=True
📍 How to use Nav2 in RViz

2D Pose Estimate: Click the button in the top toolbar and click/drag on the map to align the robot's initial position with the LiDAR scans.

2D Goal Pose: Click this button and select a destination in the warehouse. The robot will plan a path and navigate autonomously.

🧠 Step 3: Mission Control Verification
Once the Nav2 and MoveIt2 stacks are active, you can verify the autonomous coordination logic.


In a new terminal, run:


ros2 run mobile_manipulator_brain mission_control

Behavior: The node will trigger the sequence: Navigate to Pickup → Perform Pick → Navigate to Delivery → Place.


Edge Case Handling: This system is designed to retry navigation if blocked and perform a graceful halt during emergency events.






