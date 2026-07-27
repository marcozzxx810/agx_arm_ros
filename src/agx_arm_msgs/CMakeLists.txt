cmake_minimum_required(VERSION 3.5)
project(agx_arm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgxArmStatus.msg"
  "msg/AlignmentCommandTiming.msg"
  "msg/ArmStatusEvent.msg"
  "msg/DriverStateEvent.msg"
  "msg/GripperStatus.msg"
  "msg/HandStatus.msg"
  "msg/HandCmd.msg"
  "msg/HandPositionTimeCmd.msg"
  "msg/JointStateTiming.msg"
  "msg/JointAngleEvent.msg"
  "msg/MotorStateEvent.msg"
  "msg/MoveMITMsg.msg"
  "srv/GetAlignmentMetadata.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
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

ament_package()
