# Install script for directory: /home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/librclcpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so"
         OLD_RPATH "/home/deepanshu/deepanshu/osrf/ros2_rolling/install/libstatistics_collector/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcl/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcl_interfaces/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcl_yaml_param_parser/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosgraph_msgs/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/statistics_msgs/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/tracetools/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/libyaml_vendor/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rmw_implementation/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/ament_index_cpp/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcl_logging_spdlog/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcl_logging_interface/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/builtin_interfaces/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_fastrtps_c/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_fastrtps_cpp/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rmw/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/fastcdr/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_introspection_cpp/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_introspection_c/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_cpp/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_typesupport_c/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcpputils/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rosidl_runtime_c/lib:/home/deepanshu/deepanshu/osrf/ros2_rolling/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librclcpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/install/ament_package/lib/python3.8/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rclcpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rclcpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/environment" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/packages/rclcpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake/rclcppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake/rclcppExport.cmake"
         "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/CMakeFiles/Export/share/rclcpp/cmake/rclcppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake/rclcppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake/rclcppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/CMakeFiles/Export/share/rclcpp/cmake/rclcppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/CMakeFiles/Export/share/rclcpp/cmake/rclcppExport-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp/cmake" TYPE FILE FILES
    "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_core/rclcppConfig.cmake"
    "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_cmake_core/rclcppConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rclcpp" TYPE DIRECTORY FILES
    "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/include/"
    "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/include/"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rclcpp/rclcpp" TYPE FILE FILES "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_generate_version_header/rclcpp/rclcpp/version.h")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/test/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
