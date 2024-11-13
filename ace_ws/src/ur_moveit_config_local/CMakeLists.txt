cmake_minimum_required(VERSION 3.22)
project(ur_moveit_config_local)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY config launch srdf urdf
  DESTINATION share/${PROJECT_NAME}
)
