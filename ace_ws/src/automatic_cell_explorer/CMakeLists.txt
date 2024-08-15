cmake_minimum_required(VERSION 3.5)
project(automatic_cell_explorer)

# Find required packags
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

#do i need this?? compile options. Optional to add
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add cpp executables
add_executable(commander src/commander.cpp)
add_executable(octomap_subscriber src/octomap_sub.cpp)



ament_target_dependencies(
  octomap_subscriber
  octomap_msgs
  octomap
  rclcpp
)
ament_target_dependencies(
  commander
  moveit_ros_planning_interface
  rclcpp
)

# nodes
install(TARGETS commander octomap_subscriber
  DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
