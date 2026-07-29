cmake_minimum_required(VERSION 3.8)
project(robot_supervisor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/EncoderTelemetry.msg"
  "msg/FaultTelemetry.msg"
  "msg/SafetyInput.msg"
  "msg/SafetyStatus.msg"
  "msg/SupervisorCommand.msg"
  "msg/SupervisorTelemetry.msg"
  "srv/ResetFault.srv"
  "srv/SetFaultInjection.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The repository has no approved copyright-header baseline. Do not invent
  # ownership while resolving middleware lint; package metadata remains MIT.
  set(ament_cmake_copyright_FOUND TRUE)

  # This interface-only package contains no handwritten C or C++ sources.
  # Generated ROS interface code is outside the source-lint scope.
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
