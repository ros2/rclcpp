cmake_minimum_required(VERSION 2.8.3)

project(ros_dds_connext_dynamic)

find_package(ament_cmake REQUIRED)
find_package(ros_middleware_interface REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(ros_dds_cpp_dynamic_typesupport REQUIRED)

find_package(ndds_cpp REQUIRED)
set(CONNEXT_INCLUDE_DIRS ${ndds_cpp_INCLUDE_DIRS})
set(CONNEXT_LIBRARIES ${ndds_cpp_LIBRARIES})
set(CONNEXT_DEFINITIONS ${ndds_cpp_DEFINITIONS})

ament_export_dependencies(ros_middleware_interface rosidl_generator_cpp ros_dds_cpp_dynamic_typesupport)
ament_export_libraries(ros_dds_connext_dynamic dl pthread)

ament_package()

include_directories(${ros_middleware_interface_INCLUDE_DIRS} ${rosidl_generator_cpp} ${ros_dds_cpp_dynamic_typesupport} ${CONNEXT_INCLUDE_DIRS})
add_definitions(${CONNEXT_DEFINITIONS})
add_library(ros_dds_connext_dynamic SHARED src/functions.cpp)
target_link_libraries(ros_dds_connext_dynamic ${std_msgs_LIBRARIES} ${CONNEXT_LIBRARIES})

install(
  TARGETS ros_dds_connext_dynamic
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
