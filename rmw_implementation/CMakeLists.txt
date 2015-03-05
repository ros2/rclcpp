cmake_minimum_required(VERSION 2.8.3)

project(rmw_implementation NONE)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "rmw_implementation-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
