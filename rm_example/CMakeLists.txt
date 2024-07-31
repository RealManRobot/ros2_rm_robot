cmake_minimum_required(VERSION 3.8)
project(rm_example)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(
#   ${PROJECT_SOURCE_DIR}  
#   ${PROJECT_SOURCE_DIR}/include
#   ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rm_change_work_frame src/api_ChangeWorkFrame_demo.cpp)
add_executable(rm_get_state src/api_Get_Arm_State_demo.cpp)
# add_executable(rm_get_state src/api_Get_Arm_State_demo.cpp)
add_executable(movej_demo src/api_MoveJ_demo.cpp)
add_executable(movejp_demo src/api_MoveJP_demo.cpp)
add_executable(movejp_gen72_demo src/api_MoveJP_Gen72_demo.cpp)
add_executable(movel_demo src/api_MoveL_demo.cpp)
add_executable(movel_gen72_demo src/api_MoveL_Gen72_demo.cpp)

ament_target_dependencies(rm_change_work_frame rclcpp std_msgs)
ament_target_dependencies(rm_get_state rclcpp std_msgs rm_ros_interfaces)
# ament_target_dependencies(rm_get_state rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(movej_demo rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(movejp_demo rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(movel_demo rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(movejp_gen72_demo rclcpp std_msgs rm_ros_interfaces)
ament_target_dependencies(movel_gen72_demo rclcpp std_msgs rm_ros_interfaces)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
rm_change_work_frame rm_get_state movej_demo movejp_demo movel_demo movejp_gen72_demo movel_gen72_demo
  DESTINATION lib/${PROJECT_NAME})
ament_package()
