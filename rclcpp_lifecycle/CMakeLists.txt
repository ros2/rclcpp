cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  #  ${rclcpp_INCLUDE_DIRS}
  #  ${rmw_INCLUDE_DIRS}
)

set(rcl_lifecycle_sources
  src/rcl_lifecycle/lifecycle_state.c
  src/rcl_lifecycle/default_state_machine.c
  src/rcl_lifecycle/transition_map.c
)
set_source_files_properties(
  ${rcl_lifecycle_sources}
  PROPERTIES language "C"
)
add_library(
  rcl_lifecycle
  SHARED
  ${rcl_lifecycle_sources}
)
install(TARGETS rcl_lifecycle
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_library(rclcpp_lifecycle${target_suffix}
    src/rclcpp_lifecycle/lifecycle_manager.cpp)
  target_link_libraries(rclcpp_lifecycle${target_suffix}
    rcl_lifecycle
  )
  ament_target_dependencies(rclcpp_lifecycle${target_suffix}
    "rclcpp${target_suffix}"
  )

  add_executable(lifecycle_talker${target_suffix}
    src/rclcpp_lifecycle/lifecycle_talker.cpp)
  target_link_libraries(lifecycle_talker${target_suffix}
    rclcpp_lifecycle
  )
  ament_target_dependencies(lifecycle_talker${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    )
  install(TARGETS lifecycle_talker${target_suffix}
    DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
