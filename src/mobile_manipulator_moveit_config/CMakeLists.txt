cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the directories containing your configurations and launch files
install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

# Optional: If you have a 'meshes' or 'urdf' folder in this package, install them too
# install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()