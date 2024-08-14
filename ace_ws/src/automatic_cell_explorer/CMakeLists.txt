cmake_minimum_required(VERSION 3.5)
project(automatic_cell_explorer)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(commander src/commander.cpp)

target_include_directories(commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(commander PUBLIC c_std_99 cxx_std_17) 
 # Require C99 and C++17

ament_target_dependencies(
  commander
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS commander
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
