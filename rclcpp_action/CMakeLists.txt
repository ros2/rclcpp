cmake_minimum_required(VERSION 3.5)

project(rclcpp_action)

find_package(ament_cmake_ros REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(${PROJECT_NAME}_SRCS
  src/client.cpp
  src/qos.cpp
  src/server.cpp
  src/server_goal_handle.cpp
  src/types.cpp
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})

ament_target_dependencies(${PROJECT_NAME}
  "action_msgs"
  "rcl_action"
  "rclcpp"
  "rosidl_generator_c"
  "rosidl_generator_cpp")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_ACTION_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(action_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcl_action)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_generator_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_client test/test_client.cpp)
  if(TARGET test_client)
    ament_target_dependencies(test_client
      "test_msgs"
    )
    target_link_libraries(test_client
      ${PROJECT_NAME}
    )
  endif()

  ament_add_gtest(test_server test/test_server.cpp)
  if(TARGET test_server)
    ament_target_dependencies(test_server
      "test_msgs"
    )
    target_link_libraries(test_server
      ${PROJECT_NAME}
    )
  endif()

endif()

ament_package()
