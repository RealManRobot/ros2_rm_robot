cmake_minimum_required(VERSION 3.8)
project(rm_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# find dependencies

set(msg_files
  msg/Movej.msg
  # msg/Movej75.msg
  msg/Movejp.msg
  msg/Movel.msg
  msg/Movec.msg
  msg/Setrealtimepush.msg
  msg/Armsoftversion.msg
  msg/Sixforce.msg
  msg/Jointerrorcode.msg
  msg/Forcepositionmovejoint.msg
  # msg/Forcepositionmovejoint75.msg
  msg/Forcepositionmovepose.msg
  msg/Setforceposition.msg
  msg/Getallframe.msg
  msg/Jointpos.msg
  # msg/Jointpos75.msg
  msg/Cartepos.msg
  msg/Jointerrclear.msg
  msg/Gripperpick.msg
  msg/Gripperset.msg
  msg/Handangle.msg
  msg/Handforce.msg
  msg/Handposture.msg
  msg/Handseq.msg
  msg/Handspeed.msg
  msg/Armoriginalstate.msg
  msg/Armstate.msg
  msg/Liftspeed.msg
  msg/Liftheight.msg
  msg/Liftstate.msg
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
