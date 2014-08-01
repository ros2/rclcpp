cmake_minimum_required(VERSION 2.8.3)

project(ros_dds_connext_dynamic)

find_package(ament_cmake REQUIRED)
find_package(ros_middleware_interface REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

find_package(ndds_cpp REQUIRED)
#set(CONNEXT_INCLUDE_DIRS ${ndds_cpp_INCLUDE_DIRS})
set(CONNEXT_INCLUDE_DIRS
  "/home/dthomas/RTI/ndds.5.1.0/include/ndds"
  "/home/dthomas/RTI/ndds.5.1.0/include"
)
#set(CONNEXT_LIBRARIES ${ndds_cpp_LIBRARIES})
set(CONNEXT_LIBRARIES
  "/home/dthomas/RTI/ndds.5.1.0/lib/x64Linux2.6gcc4.4.5/libnddscppz.a"
  "/home/dthomas/RTI/ndds.5.1.0/lib/x64Linux2.6gcc4.4.5/libnddscz.a"
  "/home/dthomas/RTI/ndds.5.1.0/lib/x64Linux2.6gcc4.4.5/libnddscorez.a"
)
set(CONNEXT_DEFINITIONS ${ndds_cpp_DEFINITIONS})


set(_dds_vendor "$ENV{ROS_DDS_IMPLEMENTATION}")
set(_dds_type "$ENV{ROS_DDS_TYPE}")
if("${_dds_type}" STREQUAL "dynamic")
  find_package(ros_dds_cpp_dynamic_typesupport REQUIRED)
  ament_export_dependencies(ros_dds_cpp_dynamic_typesupport)
  include_directories(${ros_dds_cpp_dynamic_typesupport_INCLUDE_DIRS})
elseif("${_dds_type}" STREQUAL "static")
  if("${_dds_vendor}" STREQUAL "connext")
    find_package(ros_dds_connext_static REQUIRED)
    ament_export_definitions(${CONNEXT_DEFINITIONS})
    ament_export_dependencies(ros_dds_connext_static)
    include_directories(${ros_dds_connext_static_INCLUDE_DIRS})
  elseif("${_dds_vendor}" STREQUAL "opensplice")
    message(FATAL_ERROR "The DDS implementation '${_dds_vendor}' does not yet support the type '${_dds_type}'.")
  else()
    message(FATAL_ERROR "Set the environment variable 'ROS_DDS_IMPLEMENTATION' to either 'connext' or 'opensplice'.")
  endif()
else()
  message(FATAL_ERROR "Set the environment variable 'ROS_DDS_TYPE' to either 'dynamic' or 'static'.")
endif()


ament_export_dependencies(ros_middleware_interface rosidl_generator_cpp)

ament_export_libraries(ros_dds_connext_dynamic dl pthread)

ament_package()

include_directories(${ros_middleware_interface_INCLUDE_DIRS} ${rosidl_generator_cpp_INCLUDE_DIRS} ${CONNEXT_INCLUDE_DIRS})
link_directories("/home/dthomas/RTI/ndds.5.1.0/lib/x64Linux2.6gcc4.4.5")
add_definitions(${CONNEXT_DEFINITIONS})
add_library(ros_dds_connext_dynamic SHARED src/functions.cpp)
target_link_libraries(ros_dds_connext_dynamic ${std_msgs_LIBRARIES} ${CONNEXT_LIBRARIES})

install(
  TARGETS ros_dds_connext_dynamic
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
