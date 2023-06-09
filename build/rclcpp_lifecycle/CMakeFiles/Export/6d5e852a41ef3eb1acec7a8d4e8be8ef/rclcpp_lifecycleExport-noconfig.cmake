#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rclcpp_lifecycle::rclcpp_lifecycle" for configuration ""
set_property(TARGET rclcpp_lifecycle::rclcpp_lifecycle APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rclcpp_lifecycle::rclcpp_lifecycle PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librclcpp_lifecycle.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/librclcpp_lifecycle.dylib"
  )

list(APPEND _cmake_import_check_targets rclcpp_lifecycle::rclcpp_lifecycle )
list(APPEND _cmake_import_check_files_for_rclcpp_lifecycle::rclcpp_lifecycle "${_IMPORT_PREFIX}/lib/librclcpp_lifecycle.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
