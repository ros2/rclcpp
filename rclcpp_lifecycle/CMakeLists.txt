cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

### CPP High level library
add_library(rclcpp_lifecycle
  src/lifecycle_node.cpp
  src/managed_entity.cpp
  src/mutex_map.cpp
  src/mutex_map.hpp
  src/node_interfaces/lifecycle_node_interface.cpp
  src/state.cpp
  src/transition.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# specific order: dependents before dependencies
ament_target_dependencies(rclcpp_lifecycle
  "rclcpp"
  "rcl_lifecycle"
  "lifecycle_msgs"
  "rosidl_typesupport_cpp"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rclcpp_lifecycle PRIVATE "RCLCPP_LIFECYCLE_BUILDING_DLL")

install(TARGETS
  rclcpp_lifecycle EXPORT rclcpp_lifecycle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
  ament_lint_auto_find_test_dependencies()

  find_package(performance_test_fixture REQUIRED)

  add_performance_test(
    benchmark_lifecycle_client
    test/benchmark/benchmark_lifecycle_client.cpp)
  if(TARGET benchmark_lifecycle_client)
    target_link_libraries(benchmark_lifecycle_client ${PROJECT_NAME})
    ament_target_dependencies(benchmark_lifecycle_client rclcpp)
  endif()
  add_performance_test(
    benchmark_lifecycle_node
    test/benchmark/benchmark_lifecycle_node.cpp)
  if(TARGET benchmark_lifecycle_node)
    target_link_libraries(benchmark_lifecycle_node ${PROJECT_NAME})
    ament_target_dependencies(benchmark_lifecycle_node rclcpp)
  endif()
  add_performance_test(
    benchmark_state
    test/benchmark/benchmark_state.cpp)
  if(TARGET benchmark_state)
    target_link_libraries(benchmark_state ${PROJECT_NAME})
  endif()
  add_performance_test(
    benchmark_transition
    test/benchmark/benchmark_transition.cpp)
  if(TARGET benchmark_transition)
    target_link_libraries(benchmark_transition ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_lifecycle_node test/test_lifecycle_node.cpp TIMEOUT 120)
  if(TARGET test_lifecycle_node)
    ament_target_dependencies(test_lifecycle_node
      "rcl_lifecycle"
      "rclcpp"
      "rcutils"
    )
    target_link_libraries(test_lifecycle_node ${PROJECT_NAME} mimick)
  endif()
  ament_add_gtest(test_lifecycle_publisher test/test_lifecycle_publisher.cpp)
  if(TARGET test_lifecycle_publisher)
    ament_target_dependencies(test_lifecycle_publisher
      "rcl_lifecycle"
      "rclcpp"
      "test_msgs"
    )
    target_link_libraries(test_lifecycle_publisher ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_lifecycle_service_client test/test_lifecycle_service_client.cpp TIMEOUT 120)
  if(TARGET test_lifecycle_service_client)
    ament_target_dependencies(test_lifecycle_service_client
      "rcl_lifecycle"
      "rclcpp"
      "rcpputils"
      "rcutils"
    )
    target_link_libraries(test_lifecycle_service_client ${PROJECT_NAME} mimick)
  endif()
  ament_add_gtest(test_state_machine_info test/test_state_machine_info.cpp)
  if(TARGET test_state_machine_info)
    ament_target_dependencies(test_state_machine_info
      "rcl_lifecycle"
      "rclcpp"
    )
    target_link_libraries(test_state_machine_info ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_register_custom_callbacks test/test_register_custom_callbacks.cpp)
  if(TARGET test_register_custom_callbacks)
    ament_target_dependencies(test_register_custom_callbacks
      "rcl_lifecycle"
      "rclcpp"
    )
    target_link_libraries(test_register_custom_callbacks ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_callback_exceptions test/test_callback_exceptions.cpp)
  if(TARGET test_callback_exceptions)
    ament_target_dependencies(test_callback_exceptions
      "rcl_lifecycle"
      "rclcpp"
    )
    target_link_libraries(test_callback_exceptions ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_state_wrapper test/test_state_wrapper.cpp)
  if(TARGET test_state_wrapper)
    ament_target_dependencies(test_state_wrapper
      "rcl_lifecycle"
      "rclcpp"
    )
    target_link_libraries(test_state_wrapper ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_transition_wrapper test/test_transition_wrapper.cpp)
  if(TARGET test_transition_wrapper)
    ament_target_dependencies(test_transition_wrapper
      "rcl_lifecycle"
      "rclcpp"
      "rcutils"
    )
    target_link_libraries(test_transition_wrapper ${PROJECT_NAME} mimick)
    target_compile_definitions(test_transition_wrapper
      PUBLIC RCUTILS_ENABLE_FAULT_INJECTION
    )
  endif()
endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

# specific order: dependents before dependencies
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcl_lifecycle)
ament_export_dependencies(lifecycle_msgs)
ament_package()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
