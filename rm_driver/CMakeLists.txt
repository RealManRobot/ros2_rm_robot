cmake_minimum_required(VERSION 3.5)
project(rm_driver)

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
include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)
link_directories(${catkin_LIB_DIRS} ${PROJECT_SOURCE_DIR}/lib)

include_directories("/usr/local/lib")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(rm_driver src/rm_driver.cpp)


set(RM_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib")
 
# link_directories(${catkin_LIB_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/lib/RM_Service)
link_directories(${RM_STATIC_LIBRARY})

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0.0
)

ament_target_dependencies(rm_driver rclcpp std_msgs sensor_msgs rm_ros_interfaces)

install(TARGETS  rm_driver  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
