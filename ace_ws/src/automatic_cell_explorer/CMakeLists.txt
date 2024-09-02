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

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToNbv.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

#do i need this?? compile options. Optional to add
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locations of header files
include_directories(
 include
 ${ament_INCLUDE_DIRS}
 #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/automatic_cell_explorer/srv
)


## Declare a C++ library, the .cpp files are compiled into a library
add_library(my_cpp_library
  src/exploration_planner.cpp
  src/state_machine.cpp
)

# Link libraries to the library
target_link_libraries(my_cpp_library
  ${octomap_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES} 
)
ament_target_dependencies(my_cpp_library
  rclcpp
  sensor_msgs
  octomap_msgs
  octomap
  PCL
  pcl_conversions
  std_srvs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

# Add Google Test
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(state_machine_unit_test test/state_machine_unit_test.cpp)
if(TARGET state_machine_unit_test)
    target_link_libraries(state_machine_unit_test my_cpp_library)
endif()

# Add cpp executables, ros nodes
add_executable(state_machine_node src/main.cpp)
add_executable(move_robot_service_node src/move_robot_service.cpp)

target_link_libraries(state_machine_node
  my_cpp_library          # Your custom library
)
#Link libraries to ros node
#ament_target_dependencies(state_machine_node
#  octomap_msgs
#  octomap
#  rclcpp
#  sensor_msgs
#  pcl_conversions
#  PCL
#)
ament_target_dependencies(move_robot_service_node
  moveit_ros_planning_interface
  rclcpp
  std_srvs
  geometry_msgs 
)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(move_robot_service_node "${cpp_typesupport_target}")
target_link_libraries(state_machine_node "${cpp_typesupport_target}")


# nodes
install(TARGETS state_machine_node move_robot_service_node
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
