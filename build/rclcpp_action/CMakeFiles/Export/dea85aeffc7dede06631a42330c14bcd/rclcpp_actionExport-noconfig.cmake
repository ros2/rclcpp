#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rclcpp_action::rclcpp_action" for configuration ""
set_property(TARGET rclcpp_action::rclcpp_action APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rclcpp_action::rclcpp_action PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librclcpp_action.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/librclcpp_action.dylib"
  )

list(APPEND _cmake_import_check_targets rclcpp_action::rclcpp_action )
list(APPEND _cmake_import_check_files_for_rclcpp_action::rclcpp_action "${_IMPORT_PREFIX}/lib/librclcpp_action.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
