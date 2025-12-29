cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Added for teleop_manager
find_package(tf2 REQUIRED)

# --- Mission Control Executable ---
add_executable(mission_control src/mission_control.cpp)
ament_target_dependencies(mission_control 
  rclcpp 
  rclcpp_action 
  nav2_msgs 
  control_msgs 
  trajectory_msgs
  tf2
)

# --- Teleop Manager Executable ---
add_executable(teleop_manager src/teleop_manager.cpp)
ament_target_dependencies(teleop_manager 
  rclcpp 
  geometry_msgs 
  trajectory_msgs
)

# --- Install Executables ---
install(TARGETS
  mission_control
  teleop_manager
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install Launch and RViz Directories ---
# This allows the launch file to find your .rviz config dynamically
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()