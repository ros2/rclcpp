cmake_minimum_required(VERSION 2.8.3)

project(ros_dds_cpp_dynamic_typesupport)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ros_dds_cpp_dynamic_typesupport-extras.cmake"
)

install(
  PROGRAMS bin/ros_dds_cpp_dynamic_typesupport
  DESTINATION lib/ros_dds_cpp_dynamic_typesupport
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
