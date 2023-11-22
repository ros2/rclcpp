cmake_minimum_required(VERSION 3.5)

project(rclcpp_action)

find_package(ament_cmake_ros REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual
    -Wformat=2 -Wconversion -Wshadow -Wsign-conversion -Wcast-qual
  )
endif()

add_library(${PROJECT_NAME}
  src/client.cpp
  src/qos.cpp
  src/server.cpp
  src/server_goal_handle.cpp
  src/types.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${action_msgs_TARGETS}
  rcl::rcl
  rcl_action::rcl_action
  rclcpp::rclcpp
  rosidl_runtime_c::rosidl_runtime_c
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  rcpputils::rcpputils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_ACTION_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(action_msgs rcl_action rclcpp rosidl_runtime_c)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test/benchmark)

  ament_add_gtest(test_client test/test_client.cpp TIMEOUT 180)
  if(TARGET test_client)
    target_link_libraries(test_client
      ${PROJECT_NAME}
      mimick
      rcl::rcl
      rcl_action::rcl_action
      rclcpp::rclcpp
      rcutils::rcutils
      ${test_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_server test/test_server.cpp TIMEOUT 180)
  if(TARGET test_server)
    target_link_libraries(test_server
      ${PROJECT_NAME}
      mimick
      rcl_action::rcl_action
      rclcpp::rclcpp
      ${test_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_server_goal_handle test/test_server_goal_handle.cpp)
  if(TARGET test_server_goal_handle)
    target_link_libraries(test_server_goal_handle
      ${PROJECT_NAME}
      ${action_msgs_TARGETS}
      mimick
      rclcpp::rclcpp
      ${test_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_traits test/test_traits.cpp)
  if(TARGET test_traits)
    target_link_libraries(test_traits
      ${PROJECT_NAME}
      ${test_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_types test/test_types.cpp)
  if(TARGET test_types)
    target_link_libraries(test_types
      ${PROJECT_NAME}
      ${test_msgs_TARGETS}
    )
  endif()
endif()

ament_package()
