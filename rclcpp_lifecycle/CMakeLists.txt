cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include)

macro(targets)
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

  ### CPP High level library
  add_library(rclcpp_lifecycle${target_suffix}
    SHARED
    src/lifecycle_manager.cpp)
  target_link_libraries(rclcpp_lifecycle${target_suffix}
    ${rcl_lifecycle_LIBRARIES})
  ament_target_dependencies(rclcpp_lifecycle${target_suffix}
    "rclcpp${target_suffix}"
    #"rcl_lifecycle${target_suffix}"  This doesn't compile?
    "lifecycle_msgs")

  ### demos
  add_executable(lifecycle_talker${target_suffix}
    src/demos/lifecycle_talker.cpp)
  target_link_libraries(lifecycle_talker${target_suffix}
    rclcpp_lifecycle${target_suffix})
  add_executable(lifecycle_listener${target_suffix}
    src/demos/lifecycle_listener.cpp)
  target_link_libraries(lifecycle_listener${target_suffix}
    rclcpp_lifecycle${target_suffix})
  add_executable(lifecycle_service_client${target_suffix}
    src/demos/lifecycle_service_client.cpp)
  target_link_libraries(lifecycle_service_client${target_suffix}
    rclcpp_lifecycle${target_suffix})

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
