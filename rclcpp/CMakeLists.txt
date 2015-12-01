cmake_minimum_required(VERSION 2.8.3)

project(rclcpp)

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

include_directories(include)

set(${PROJECT_NAME}_SRCS
  src/rclcpp/any_executable.cpp
  src/rclcpp/callback_group.cpp
  src/rclcpp/client.cpp
  src/rclcpp/context.cpp
  src/rclcpp/contexts/default_context.cpp
  src/rclcpp/executor.cpp
  src/rclcpp/executors.cpp
  src/rclcpp/executors/multi_threaded_executor.cpp
  src/rclcpp/executors/single_threaded_executor.cpp
  src/rclcpp/intra_process_manager.cpp
  src/rclcpp/intra_process_manager_impl.cpp
  src/rclcpp/memory_strategies.cpp
  src/rclcpp/memory_strategy.cpp
  src/rclcpp/parameter.cpp
  src/rclcpp/parameter_client.cpp
  src/rclcpp/parameter_service.cpp
  src/rclcpp/publisher.cpp
  src/rclcpp/node.cpp
  src/rclcpp/service.cpp
  src/rclcpp/subscription.cpp
  src/rclcpp/timer.cpp
  src/rclcpp/utilities.cpp
)
if(WIN32)
  list(APPEND ${PROJECT_NAME}_SRCS src/rclcpp/windows_helper.cpp)
endif()
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})
ament_target_dependencies(
  ${PROJECT_NAME}
  "rcl_interfaces"
  "rmw"
  "rosidl_generator_cpp"
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# It also avoids using certain parts of the code that the consuming code must
# use, but which the library cannot consume itself.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RCLCPP_BUILDING_LIBRARY")
if(APPLE)
  # Since the rmw_* symbols are unresolved at the time of building librclcpp,
  # tell the linker on OS X to dynamically look them up at runtime.
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

ament_export_dependencies(rcl_interfaces)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_generator_cpp)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_function_traits test/test_function_traits.cpp)
  if(TARGET test_function_traits)
    target_include_directories(test_function_traits PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
    )
  endif()
  ament_add_gtest(test_mapped_ring_buffer test/test_mapped_ring_buffer.cpp)
  if(TARGET test_mapped_ring_buffer)
    target_include_directories(test_mapped_ring_buffer PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
    )
  endif()
  ament_add_gtest(test_intra_process_manager test/test_intra_process_manager.cpp)
  if(TARGET test_intra_process_manager)
    target_include_directories(test_intra_process_manager PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
    )
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

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
