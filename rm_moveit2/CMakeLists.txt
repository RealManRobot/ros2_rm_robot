cmake_minimum_required(VERSION 3.8)
project(rm_moveit2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(simple_moveit_node
  src/simple_moveit_node.cpp
  src/linear_motion_node.cpp
)
target_include_directories(simple_moveit_node PRIVATE include)
ament_target_dependencies(
  simple_moveit_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)



install(TARGETS
  simple_moveit_node

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
