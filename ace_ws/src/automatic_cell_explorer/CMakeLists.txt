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

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

#specify locations of header files
include_directories(
 include
 ${ament_INCLUDE_DIRS}
)

## Declare a C++ library, the .cpp files are compiled into a library
add_library(my_cpp_library
  src/state_machine.cpp
  # Add more source files as needed
)

# Add Google Test
find_package(ament_cmake_gtest REQUIRED)

# Add a test executable
ament_add_gtest(state_machine_unit_test test/state_machine_unit_test.cpp)
if(TARGET state_machine_unit_test)
    target_link_libraries(state_machine_unit_test my_cpp_library)
endif()

# Add cpp executables, ros nodes
add_executable(commander src/commander.cpp)
add_executable(octomap_processor src/octomap_processor.cpp)
add_executable(planner src/planner.cpp)
add_executable(move_robot src/move_robot.cpp)


#Link libraries to ros node
ament_target_dependencies(
  octomap_processor
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
  tf2_geometry_msgs
  tf2
)

ament_target_dependencies(
  move_robot
  rclcpp
  moveit_ros_planning_interface
  std_srvs
  geometry_msgs 
)

#difference between this and ament_target_dependencies??
target_link_libraries(octomap_processor ${PCL_LIBRARIES})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(move_robot "${cpp_typesupport_target}")
target_link_libraries(planner "${cpp_typesupport_target}")

#add_dependencies()?

# nodes
install(TARGETS commander octomap_processor planner move_robot
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
