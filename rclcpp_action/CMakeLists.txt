cmake_minimum_required(VERSION 3.5)

project(rclcpp_action)

find_package(ament_cmake_ros REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual
    -Wformat=2 -Wconversion -Wshadow -Wsign-conversion -Wcast-qual
  )
endif()

set(${PROJECT_NAME}_SRCS
  src/client.cpp
  src/qos.cpp
  src/server.cpp
  src/server_goal_handle.cpp
  src/types.cpp
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME}
  "action_msgs"
  "rcl_action"
  "rclcpp"
  "rcpputils"
  "rosidl_runtime_c"
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

# specific order: dependents before dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(action_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcl_action)
ament_export_dependencies(rosidl_runtime_c)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test/benchmark)

  ament_add_gtest(test_client test/test_client.cpp TIMEOUT 180)
  if(TARGET test_client)
    ament_target_dependencies(test_client
      "rcutils"
      "test_msgs"
    )
    target_link_libraries(test_client
      ${PROJECT_NAME}
      mimick
    )
  endif()

  ament_add_gtest(test_server test/test_server.cpp TIMEOUT 180)
  if(TARGET test_server)
    ament_target_dependencies(test_server
      "rcpputils"
      "rcutils"
      "test_msgs"
    )
    target_link_libraries(test_server
      ${PROJECT_NAME}
      mimick
    )
  endif()

  ament_add_gtest(test_server_goal_handle test/test_server_goal_handle.cpp)
  if(TARGET test_server_goal_handle)
    ament_target_dependencies(test_server_goal_handle
      "rcutils"
      "test_msgs"
    )
    target_link_libraries(test_server_goal_handle
      ${PROJECT_NAME}
      mimick
    )
  endif()

  ament_add_gtest(test_traits test/test_traits.cpp)
  if(TARGET test_traits)
    ament_target_dependencies(test_traits
      "test_msgs"
    )
    target_link_libraries(test_traits
      ${PROJECT_NAME}
    )
  endif()

  ament_add_gtest(test_types test/test_types.cpp)
  if(TARGET test_types)
    ament_target_dependencies(test_types
      "test_msgs"
    )
    target_link_libraries(test_types
      ${PROJECT_NAME}
    )
  endif()
endif()

ament_package()
