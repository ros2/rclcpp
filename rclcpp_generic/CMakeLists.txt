cmake_minimum_required(VERSION 3.5)
project(rclcpp_generic)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rclcpp_generic/generic_publisher.cpp
  src/rclcpp_generic/generic_subscription.cpp
  src/rclcpp_generic/typesupport_helpers.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rcl
  rclcpp
  rcutils
  rcpputils
  rosidl_runtime_cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RCLCPP_GENERIC_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()

function(create_tests_for_rmw_implementation)
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})
  ament_add_gmock(test_pubsub${target_suffix}
    test/rclcpp_generic/test_pubsub.cpp
    ENV ${rmw_implementation_env_var}
  )
  target_link_libraries(test_pubsub${target_suffix} ${PROJECT_NAME})
  target_include_directories(test_pubsub${target_suffix} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rclcpp_generic>)
  ament_target_dependencies(test_pubsub${target_suffix} rcpputils rosidl_typesupport_cpp test_msgs)
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  # explicitly disable cppcheck
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # This test is independent from the rmw implementation
  ament_add_gmock(test_typesupport_helpers
    test/rclcpp_generic/test_typesupport_helpers.cpp
  )
  target_link_libraries(test_typesupport_helpers ${PROJECT_NAME})
  target_include_directories(test_typesupport_helpers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rclcpp_generic>)
  ament_target_dependencies(test_typesupport_helpers rcpputils)


  call_for_each_rmw_implementation(create_tests_for_rmw_implementation)
endif()

ament_package()
