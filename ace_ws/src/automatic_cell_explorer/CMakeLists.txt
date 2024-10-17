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
find_package(Eigen3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Execute.srv"
  DEPENDENCIES moveit_msgs
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
 ${EIGEN_INCLUDE_DIR}
 #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/automatic_cell_explorer/srv
)


## Declare a C++ library, the .cpp files are compiled into a library
add_library(my_cpp_library
  src/visualize.cpp
  src/ep_factory.cpp
  src/state_machine.cpp
  src/moveit_interface.cpp
  src/octomap_processor.cpp
  src/move_robot_service.cpp
  src/exploration_planner/raycast.cpp
  src/exploration_planner/exploration_planner.cpp
  src/exploration_planner/exploration_planners/demo_exploration_planner.cpp
  src/exploration_planner/exploration_planners/random_exploration_planner.cpp
  src/exploration_planner/exploration_planners/exploration_planner_v2.cpp
  src/exploration_planner/exploration_planners/exploration_planner_v3.cpp
)

# Link libraries to the library
target_link_libraries(my_cpp_library
  ${octomap_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES} 
  Eigen3::Eigen

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
  moveit_msgs
  moveit_visual_tools
)

rosidl_target_interfaces(my_cpp_library
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add Google Test
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(state_machine_unit_test test/state_machine_unit_test.cpp)
# if(TARGET state_machine_unit_test)
#     target_link_libraries(state_machine_unit_test my_cpp_library)
# endif()

# ament_add_gtest(exploration_planner_unit_test test/exploration_planner_unit_test.cpp)
# if(TARGET exploration_planner_unit_test)
#     target_link_libraries(exploration_planner_unit_test my_cpp_library)
# endif()

# Add cpp executables, ros nodes
add_executable(ace src/main.cpp)



target_link_libraries(ace
  my_cpp_library          # Your custom library
)

#Link libraries to ros node
#ament_target_dependencies(ace
#  octomap_msgs
#  octomap
#  rclcpp
#  sensor_msgs
#  pcl_conversions
#  PCL
#)

ament_target_dependencies(ace
  moveit_ros_planning_interface
  rclcpp
  std_srvs
  geometry_msgs 
)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(ace "${cpp_typesupport_target}")






# nodes
install(TARGETS ace 
  DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

ament_package()
