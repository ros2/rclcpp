cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include)

set(rcl_lifecycle_sources
  src/rcl_lifecycle/default_state_machine.c
  src/rcl_lifecycle/rcl_lifecycle.c
  src/rcl_lifecycle/transition_map.c)
set_source_files_properties(
  ${rcl_lifecycle_sources}
  PROPERTIES language "C")

macro(targets)
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  get_rmw_typesupport(typesupport_impls_c "${rmw_implementation}" LANGUAGE "c")
  get_rmw_typesupport(typesupport_impls_cpp "${rmw_implementation}" LANGUAGE "cpp")

  ### C-Library depending only on RCL
  add_library(
    rcl_lifecycle${target_suffix}
    SHARED
    ${rcl_lifecycle_sources})

  ament_target_dependencies(rcl_lifecycle${target_suffix}
    "rcl${target_suffix}"
    "std_msgs"
    "lifecycle_msgs")

  install(TARGETS rcl_lifecycle${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ### CPP High level library
  add_library(rclcpp_lifecycle${target_suffix}
    SHARED
    src/rclcpp_lifecycle/lifecycle_manager.cpp)
  target_link_libraries(rclcpp_lifecycle${target_suffix}
    rcl_lifecycle${target_suffix})
  ament_target_dependencies(rclcpp_lifecycle${target_suffix}
    "rclcpp${target_suffix}"
    "lifecycle_msgs")

  ### demos
  add_executable(lifecycle_talker${target_suffix}
    src/demos/lifecycle_talker.cpp)
  target_link_libraries(lifecycle_talker${target_suffix}
    rclcpp_lifecycle${target_suffix})
  ament_target_dependencies(lifecycle_talker${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs")
  add_executable(lifecycle_listener${target_suffix}
    src/demos/lifecycle_listener.cpp)
  target_link_libraries(lifecycle_listener${target_suffix}
    rclcpp_lifecycle${target_suffix})
  ament_target_dependencies(lifecycle_listener${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "lifecycle_msgs")
  add_executable(lifecycle_service_client${target_suffix}
    src/demos/lifecycle_service_client.cpp)
  target_link_libraries(lifecycle_service_client${target_suffix}
    rclcpp_lifecycle${target_suffix})
  ament_target_dependencies(lifecycle_service_client${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "lifecycle_msgs")

  install(TARGETS
    rclcpp_lifecycle${target_suffix}
    lifecycle_talker${target_suffix}
    lifecycle_listener${target_suffix}
    lifecycle_service_client${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
