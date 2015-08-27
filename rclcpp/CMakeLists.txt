cmake_minimum_required(VERSION 2.8.3)

project(rclcpp)

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)

ament_export_dependencies(rmw)
ament_export_dependencies(rcl_interfaces)

ament_export_include_directories(include)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(rmw REQUIRED)

  if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
  endif()

  include_directories(include)

  ament_add_gtest(test_mapped_ring_buffer test/test_mapped_ring_buffer.cpp)
  ament_add_gtest(test_intra_process_manager test/test_intra_process_manager.cpp)
  if(TARGET test_intra_process_manager)
    target_include_directories(test_intra_process_manager PUBLIC
      "${rcl_interfaces_INCLUDE_DIRS}"
      "${rmw_INCLUDE_DIRS}")
  endif()
endif()

ament_package(
  CONFIG_EXTRAS rclcpp-extras.cmake
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY src/
  DESTINATION src/rclcpp
)
