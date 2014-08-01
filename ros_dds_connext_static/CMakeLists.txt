cmake_minimum_required(VERSION 2.8.3)

project(ros_dds_connext_static)

find_package(ament_cmake REQUIRED)
find_package(ros_middleware_interface REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_generator_dds_connext_cpp REQUIRED)

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

ament_export_definitions(${CONNEXT_DEFINITIONS})
ament_export_dependencies(ros_middleware_interface rosidl_generator_cpp rosidl_generator_dds_connext_cpp)
ament_export_include_directories(include ${CONNEXT_INCLUDE_DIRS})
ament_export_libraries(ros_dds_connext_static dl pthread)

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "ros_dds_connext_static-extras.cmake"
)

include_directories(include ${ros_middleware_interface_INCLUDE_DIRS} ${rosidl_generator_cpp_INCLUDE_DIRS} ${rosidl_generator_dds_connext_cpp_INCLUDE_DIRS} ${CONNEXT_INCLUDE_DIRS})
link_directories("/home/dthomas/RTI/ndds.5.1.0/lib/x64Linux2.6gcc4.4.5")
add_definitions(${CONNEXT_DEFINITIONS})
add_library(ros_dds_connext_static SHARED src/functions.cpp)
target_link_libraries(ros_dds_connext_static ${std_msgs_LIBRARIES} ${rosidl_generator_dds_connext_cpp_LIBRARIES} ${CONNEXT_LIBRARIES})

install(
  PROGRAMS bin/ros_dds_connext_static
  DESTINATION lib/ros_dds_connext_static
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ros_dds_connext_static
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
