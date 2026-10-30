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

set(RM_API_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libapi_cpp.so")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # rm_define.h, rm_interface*.h, rm_service.h and rm_version.h are shipped
  # with the versioned RealMan SDK binary and must remain byte-compatible with
  # the vendor release. Keep them out of source-format checks and lint the ROS
  # adapter itself explicitly.
  set(RM_DRIVER_VENDOR_HEADERS
    include/rm_driver/rm_define.h
    include/rm_driver/rm_interface.h
    include/rm_driver/rm_interface_global.h
    include/rm_driver/rm_service.h
    include/rm_driver/rm_version.h
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${RM_DRIVER_VENDOR_HEADERS})
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_cpplint(
    TESTNAME cpplint_ros_adapter
    MAX_LINE_LENGTH 120
    include/rm_driver/rm_driver.h
    src/rm_driver.cpp
    FILTERS
      -readability/fn_size
      -whitespace/line_length
  )
  ament_uncrustify(
    TESTNAME uncrustify_ros_adapter
    include/rm_driver/rm_driver.h
    src/rm_driver.cpp
  )
endif()
add_executable(rm_driver src/rm_driver.cpp)
target_link_libraries(rm_driver ${RM_API_LIBRARY})

set_target_properties(rm_driver PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

ament_target_dependencies(
  rm_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rm_ros_interfaces
)

install(TARGETS  rm_driver  DESTINATION lib/${PROJECT_NAME})
install(FILES ${RM_API_LIBRARY} DESTINATION lib)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
