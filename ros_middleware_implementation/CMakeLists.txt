cmake_minimum_required(VERSION 2.8.3)

project(ros_middleware_implementation NONE)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "ros_middleware_implementation-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
