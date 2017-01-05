cmake_minimum_required(VERSION 3.5)

project(rclcpp)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(${PROJECT_NAME}_SRCS
  src/rclcpp/any_executable.cpp
  src/rclcpp/callback_group.cpp
  src/rclcpp/client.cpp
  src/rclcpp/context.cpp
  src/rclcpp/contexts/default_context.cpp
  src/rclcpp/event.cpp
  src/rclcpp/exceptions.cpp
  src/rclcpp/executor.cpp
  src/rclcpp/executors.cpp
  src/rclcpp/executors/multi_threaded_executor.cpp
  src/rclcpp/executors/single_threaded_executor.cpp
  src/rclcpp/graph_listener.cpp
  src/rclcpp/intra_process_manager.cpp
  src/rclcpp/intra_process_manager_impl.cpp
  src/rclcpp/memory_strategies.cpp
  src/rclcpp/memory_strategy.cpp
  src/rclcpp/node.cpp
  src/rclcpp/node_interfaces/node_base.cpp
  src/rclcpp/node_interfaces/node_graph.cpp
  src/rclcpp/node_interfaces/node_parameters.cpp
  src/rclcpp/node_interfaces/node_services.cpp
  src/rclcpp/node_interfaces/node_timers.cpp
  src/rclcpp/node_interfaces/node_topics.cpp
  src/rclcpp/parameter.cpp
  src/rclcpp/parameter_client.cpp
  src/rclcpp/parameter_service.cpp
  src/rclcpp/publisher.cpp
  src/rclcpp/service.cpp
  src/rclcpp/subscription.cpp
  src/rclcpp/timer.cpp
  src/rclcpp/type_support.cpp
  src/rclcpp/utilities.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS})
ament_target_dependencies(${PROJECT_NAME}
  "rcl"
  "rosidl_generator_cpp"
  "rosidl_typesupport_cpp")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
ament_export_dependencies(rosidl_generator_cpp)
ament_export_dependencies(rosidl_typesupport_c)
ament_export_dependencies(rosidl_typesupport_cpp)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(rmw_implementation_cmake REQUIRED)

  ament_add_gtest(test_function_traits test/test_function_traits.cpp)
  if(TARGET test_function_traits)
    target_include_directories(test_function_traits PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
  endif()
  ament_add_gtest(test_mapped_ring_buffer test/test_mapped_ring_buffer.cpp)
  if(TARGET test_mapped_ring_buffer)
    target_include_directories(test_mapped_ring_buffer PUBLIC
      ${rcl_INCLUDE_DIRS}
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
  endif()
  ament_add_gtest(test_intra_process_manager test/test_intra_process_manager.cpp)
  if(TARGET test_intra_process_manager)
    target_include_directories(test_intra_process_manager PUBLIC
      ${rcl_INCLUDE_DIRS}
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
  endif()
  ament_add_gtest(test_rate test/test_rate.cpp
    ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
  if(TARGET test_rate)
    target_include_directories(test_rate PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_rate
      ${PROJECT_NAME}
    )
  endif()
  ament_add_gtest(test_find_weak_nodes test/test_find_weak_nodes.cpp)
  if(TARGET test_find_weak_nodes)
    target_include_directories(test_find_weak_nodes PUBLIC
      ${rcl_INCLUDE_DIRS}
    )
    target_link_libraries(test_find_weak_nodes ${PROJECT_NAME})
  endif()

  get_default_rmw_implementation(default_rmw)
  find_package(${default_rmw} REQUIRED)
  get_rmw_typesupport(typesupport_impls_cpp "${default_rmw}" LANGUAGE "cpp")
  get_rmw_typesupport(typesupport_impls_c "${default_rmw}" LANGUAGE "c")
  set(mock_msg_files
    "test/mock_msgs/srv/Mock.srv")
  rosidl_generate_interfaces(mock_msgs
    ${mock_msg_files}
    LIBRARY_NAME "rclcpp"
    SKIP_INSTALL)

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  ament_add_gtest(test_externally_defined_services test/test_externally_defined_services.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_externally_defined_services)
    target_include_directories(test_externally_defined_services PUBLIC
      ${rcl_INCLUDE_DIRS}
    )
    target_link_libraries(test_externally_defined_services ${PROJECT_NAME})
    foreach(typesupport_impl_cpp ${typesupport_impls_cpp})
      rosidl_target_interfaces(test_externally_defined_services
        mock_msgs ${typesupport_impl_cpp})
    endforeach()
    foreach(typesupport_impl_c ${typesupport_impls_c})
      rosidl_target_interfaces(test_externally_defined_services
        mock_msgs ${typesupport_impl_c})
    endforeach()
  endif()
endif()

ament_package(
  CONFIG_EXTRAS rclcpp-extras.cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
