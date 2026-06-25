cmake_minimum_required(VERSION 3.5)
project(rm_eco63_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config srdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
