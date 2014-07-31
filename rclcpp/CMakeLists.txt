cmake_minimum_required(VERSION 2.8.3)

project(rclcpp)

find_package(ament_cmake REQUIRED)
find_package(ros_middleware_interface REQUIRED)

ament_export_dependencies(ros_dds_connext_dynamic ros_middleware_interface)
ament_export_include_directories(include)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
