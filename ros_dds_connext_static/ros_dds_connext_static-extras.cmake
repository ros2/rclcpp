# copied from ros_dds_connext_static/ros_dds_connext_static-extras.cmake

find_package(ament_cmake_core REQUIRED)
# TODO
# instead of being an extension for "rosidl_generate_interfaces"
# this should be an extension of "rosidl_generator_cpp"
# which can then ensure that there is only one
ament_register_extension("rosidl_generate_interfaces" "ros_dds_connext_static"
  "ros_dds_connext_static_generate_interfaces.cmake")

set(ros_dds_connext_static_BIN "${ros_dds_connext_static_DIR}/../../../lib/ros_dds_connext_static/ros_dds_connext_static")
set(ros_dds_connext_static_TEMPLATE_DIR "${ros_dds_connext_static_DIR}/../resource")
