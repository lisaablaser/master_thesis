cmake_minimum_required(VERSION 3.5)
project(automatic_cell_explorer)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

ament_package()
