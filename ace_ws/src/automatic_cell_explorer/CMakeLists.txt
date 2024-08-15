cmake_minimum_required(VERSION 3.5)
project(automatic_cell_explorer)

# Find required packags
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToNbv.srv"
  DEPENDENCIES geometry_msgs
)


ament_export_dependencies(rosidl_default_runtime)

#do i need this?? compile options. Optional to add
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add cpp executables
add_executable(commander src/commander.cpp)
add_executable(octomap_subscriber src/octomap_sub.cpp)
add_executable(planner src/planner.cpp)
add_executable(move_robot src/move_robot.cpp)


ament_target_dependencies(
  octomap_subscriber
  octomap_msgs
  octomap
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)
ament_target_dependencies(
  commander
  moveit_ros_planning_interface
  rclcpp
)

ament_target_dependencies(
  planner
  rclcpp
  sensor_msgs
  octomap_msgs
  pcl_conversions
  PCL
  std_srvs
  geometry_msgs
)

ament_target_dependencies(
  move_robot
  rclcpp
  moveit_ros_planning_interface
  std_srvs
  geometry_msgs 
)

target_link_libraries(octomap_subscriber ${PCL_LIBRARIES})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(move_robot "${cpp_typesupport_target}")
target_link_libraries(planner "${cpp_typesupport_target}")


# nodes
install(TARGETS commander octomap_subscriber planner move_robot
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
