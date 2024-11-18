cmake_minimum_required(VERSION 3.5)
project(cartographer_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# TODO(jwallace42): This is a work around for https://github.com/ros2/rosidl_typesupport_fastrtps/issues/28
add_compile_options(-Wno-error=deprecated)

rosidl_generate_interfaces(cartographer_msgs
  "msg/Posture.msg"
  "srv/Command.srv"
  DEPENDENCIES builtin_interfaces std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
