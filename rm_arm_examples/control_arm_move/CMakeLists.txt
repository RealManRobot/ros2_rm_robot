cmake_minimum_required(VERSION 3.8)
project(control_arm_move)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(move_demo src/api_Move_demo.cpp)
add_executable(slight_move_demo src/api_Slight_Move_demo.cpp)

ament_target_dependencies(move_demo rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(slight_move_demo rclcpp std_msgs rm_ros_interfaces)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
move_demo slight_move_demo
  DESTINATION lib/${PROJECT_NAME})
ament_package()
