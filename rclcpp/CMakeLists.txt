cmake_minimum_required(VERSION 2.8.3)

project(rclcpp)

find_package(ament_cmake REQUIRED)
find_package(ros_middleware_interface REQUIRED)

ament_export_dependencies(ros_middleware_interface)

set(_dds_vendor "$ENV{ROS_DDS_IMPLEMENTATION}")
set(_dds_type "$ENV{ROS_DDS_TYPE}")

if(NOT "${_dds_type}" STREQUAL "dynamic" AND NOT "${_dds_type}" STREQUAL "static")
  message(FATAL_ERROR "Set the environment variable 'ROS_DDS_TYPE' to either 'dynamic' or 'static'.")
endif()

if("${_dds_vendor}" STREQUAL "connext")
  if("${_dds_type}" STREQUAL "dynamic")
    ament_export_dependencies(ros_dds_connext_dynamic)
  elseif("${_dds_type}" STREQUAL "static")
    ament_export_dependencies(rosidl_generator_dds_connext_cpp ros_dds_connext_static)
  endif()
elseif("${_dds_vendor}" STREQUAL "opensplice")
  if("${_dds_type}" STREQUAL "dynamic")
    message(FATAL_ERROR "The DDS implementation '${_dds_vendor}' does not support the type '${_dds_type}'.")
  elseif("${_dds_type}" STREQUAL "static")
    ament_export_dependencies(rosidl_generator_dds_opensplice_cpp)
  endif()
else()
  message(FATAL_ERROR "Set the environment variable 'ROS_DDS_IMPLEMENTATION' to either 'connext' or 'opensplice'.")
endif()

ament_export_include_directories(include)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)
