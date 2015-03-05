cmake_minimum_required(VERSION 2.8.3)

project(rclcpp)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(rmw)

ament_export_include_directories(include)

ament_package(
  CONFIG_EXTRAS rclcpp-extras.cmake
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY src/
  DESTINATION src/rclcpp
)
