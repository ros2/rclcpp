# copied from ros_dds_cpp_dynamic_typesupport/ros_dds_cpp_dynamic_typesupport-extras.cmake

find_package(ament_cmake_core REQUIRED)
# TODO
# instead of being an extension for "rosidl_generate_interfaces"
# this should be an extension of "rosidl_generator_cpp"
# which can then ensure that there is only one
ament_register_extension("rosidl_generate_interfaces" "ros_dds_cpp_dynamic_typesupport"
  "ros_dds_cpp_dynamic_typesupport_generate_interfaces.cmake")

set(ros_dds_cpp_dynamic_typesupport_BIN "${ros_dds_cpp_dynamic_typesupport_DIR}/../../../lib/ros_dds_cpp_dynamic_typesupport/ros_dds_cpp_dynamic_typesupport")
set(ros_dds_cpp_dynamic_typesupport_TEMPLATE_DIR "${ros_dds_cpp_dynamic_typesupport_DIR}/../resource")
