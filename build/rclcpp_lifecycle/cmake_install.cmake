# Install script for directory: /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp_lifecycle

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/install/rclcpp_lifecycle")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/librclcpp_lifecycle.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp_lifecycle.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp_lifecycle.dylib")
    execute_process(COMMAND /usr/bin/install_name_tool
      -delete_rpath "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/install/rclcpp/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcl_lifecycle/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/libstatistics_collector/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/lifecycle_msgs/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcl/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcl_logging_interface/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcl_yaml_param_parser/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rmw_implementation/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/ament_index_cpp/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/service_msgs/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_fastrtps_c/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_introspection_cpp/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_introspection_c/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_fastrtps_cpp/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/fastcdr/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rmw/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_dynamic_typesupport/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_cpp/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_c/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcpputils/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rosidl_runtime_c/lib"
      -delete_rpath "/Users/cursedrock17/ros2_rolling/install/rcutils/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp_lifecycle.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp_lifecycle.dylib")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rclcpp_lifecycle" TYPE DIRECTORY FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp_lifecycle/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/ros2_rolling/build/ament_package/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rclcpp_lifecycle")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rclcpp_lifecycle")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/ros2_rolling/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/ros2_rolling/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/environment" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_index/share/ament_index/resource_index/packages/rclcpp_lifecycle")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake/rclcpp_lifecycleExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake/rclcpp_lifecycleExport.cmake"
         "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/CMakeFiles/Export/6d5e852a41ef3eb1acec7a8d4e8be8ef/rclcpp_lifecycleExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake/rclcpp_lifecycleExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake/rclcpp_lifecycleExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/CMakeFiles/Export/6d5e852a41ef3eb1acec7a8d4e8be8ef/rclcpp_lifecycleExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/CMakeFiles/Export/6d5e852a41ef3eb1acec7a8d4e8be8ef/rclcpp_lifecycleExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle/cmake" TYPE FILE FILES
    "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_core/rclcpp_lifecycleConfig.cmake"
    "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/ament_cmake_core/rclcpp_lifecycleConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp_lifecycle" TYPE FILE FILES "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp_lifecycle/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp_lifecycle/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
