message(" - ros_dds_cpp_dynamic_typesupport_generate_interfaces.cmake")
message("   - target: ${rosidl_generate_interfaces_TARGET}")
message("   - interface files: ${rosidl_generate_interfaces_IDL_FILES}")
message("   - dependency package names: ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES}")

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/ros_dds_cpp_dynamic_typesupport/${PROJECT_NAME}")
set(_generated_files "")
foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(name "${_idl_file}" NAME_WE)
  list(APPEND _generated_files
    "${_output_path}/${name}_TypeSupport.h"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

message("   - generated files: ${_generated_files}")
message("   - dependencies: ${_dependencies}")

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${ros_dds_cpp_dynamic_typesupport_BIN}
  --pkg-name ${PROJECT_NAME}
  --interface-files ${rosidl_generate_interfaces_IDL_FILES}
  --deps ${_dependencies}
  --output-dir ${_output_path}
  --template-dir ${ros_dds_cpp_dynamic_typesupport_TEMPLATE_DIR}
  DEPENDS
  ${ros_dds_cpp_dynamic_typesupport_BIN}
  ${ros_dds_cpp_dynamic_typesupport_DIR}/../../../${PYTHON_INSTALL_DIR}/ros_dds_cpp_dynamic_typesupport/__init__.py
  ${ros_dds_cpp_dynamic_typesupport_TEMPLATE_DIR}/msg_TypeSupport.h.template
  ${rosidl_generate_interfaces_IDL_FILES}
  ${_dependency_files}
  COMMENT "Generating C++ code for interfaces"
  VERBATIM
)

add_custom_target(
  ${rosidl_generate_interfaces_TARGET}_cpp_dynamic_typesupport
  DEPENDS
  ${_generated_files}
)
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}_cpp_dynamic_typesupport
)

install(
  FILES ${_generated_files}
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories(include)
