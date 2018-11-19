cmake_minimum_required(VERSION 3.5)

project(rclcpp)

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(${PROJECT_NAME}_SRCS
  src/rclcpp/any_executable.cpp
  src/rclcpp/callback_group.cpp
  src/rclcpp/client.cpp
  src/rclcpp/clock.cpp
  src/rclcpp/context.cpp
  src/rclcpp/contexts/default_context.cpp
  src/rclcpp/duration.cpp
  src/rclcpp/event.cpp
  src/rclcpp/exceptions.cpp
  src/rclcpp/executor.cpp
  src/rclcpp/executors.cpp
  src/rclcpp/expand_topic_or_service_name.cpp
  src/rclcpp/executors/multi_threaded_executor.cpp
  src/rclcpp/executors/single_threaded_executor.cpp
  src/rclcpp/graph_listener.cpp
  src/rclcpp/init_options.cpp
  src/rclcpp/intra_process_manager.cpp
  src/rclcpp/intra_process_manager_impl.cpp
  src/rclcpp/logger.cpp
  src/rclcpp/memory_strategies.cpp
  src/rclcpp/memory_strategy.cpp
  src/rclcpp/node.cpp
  src/rclcpp/node_interfaces/node_base.cpp
  src/rclcpp/node_interfaces/node_clock.cpp
  src/rclcpp/node_interfaces/node_graph.cpp
  src/rclcpp/node_interfaces/node_logging.cpp
  src/rclcpp/node_interfaces/node_parameters.cpp
  src/rclcpp/node_interfaces/node_services.cpp
  src/rclcpp/node_interfaces/node_timers.cpp
  src/rclcpp/node_interfaces/node_topics.cpp
  src/rclcpp/node_interfaces/node_waitables.cpp
  src/rclcpp/parameter.cpp
  src/rclcpp/parameter_value.cpp
  src/rclcpp/parameter_client.cpp
  src/rclcpp/parameter_events_filter.cpp
  src/rclcpp/parameter_map.cpp
  src/rclcpp/parameter_service.cpp
  src/rclcpp/publisher.cpp
  src/rclcpp/service.cpp
  src/rclcpp/subscription.cpp
  src/rclcpp/time.cpp
  src/rclcpp/time_source.cpp
  src/rclcpp/timer.cpp
  src/rclcpp/type_support.cpp
  src/rclcpp/utilities.cpp
  src/rclcpp/waitable.cpp
)

# "watch" template for changes
configure_file(
  "resource/logging.hpp.em"
  "logging.hpp.em.watch"
  COPYONLY
)
# generate header with logging macros
set(python_code
  "import em"
  "em.invoke(['-o', 'include/rclcpp/logging.hpp', '${CMAKE_CURRENT_SOURCE_DIR}/resource/logging.hpp.em'])")
string(REPLACE ";" "$<SEMICOLON>" python_code "${python_code}")
add_custom_command(OUTPUT include/rclcpp/logging.hpp
  COMMAND ${CMAKE_COMMAND} -E make_directory "include/rclcpp"
  COMMAND ${PYTHON_EXECUTABLE} ARGS -c "${python_code}"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/logging.hpp.em.watch"
  COMMENT "Expanding logging.hpp.em"
  VERBATIM
)
list(APPEND ${PROJECT_NAME}_SRCS
  include/rclcpp/logging.hpp)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})
# specific order: dependents before dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rcl"
  "rcl_yaml_param_parser"
  "builtin_interfaces"
  "rosgraph_msgs"
  "rosidl_typesupport_cpp"
  "rosidl_generator_cpp")

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

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rosgraph_msgs)
ament_export_dependencies(rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_typesupport_c)
ament_export_dependencies(rosidl_generator_cpp)
ament_export_dependencies(rcl_yaml_param_parser)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(rmw_implementation_cmake REQUIRED)

  ament_add_gtest(test_client test/test_client.cpp)
  if(TARGET test_client)
    target_include_directories(test_client PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_client ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_expand_topic_or_service_name test/test_expand_topic_or_service_name.cpp)
  if(TARGET test_expand_topic_or_service_name)
    target_include_directories(test_expand_topic_or_service_name PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_expand_topic_or_service_name ${PROJECT_NAME})
  endif()
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
  ament_add_gtest(test_node test/test_node.cpp)
  if(TARGET test_node)
    target_include_directories(test_node PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_node ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_node_global_args test/test_node_global_args.cpp)
  if(TARGET test_node_global_args)
    target_include_directories(test_node_global_args PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_node_global_args ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_node_initial_parameters test/test_node_initial_parameters.cpp)
  if(TARGET test_node_initial_parameters)
    target_link_libraries(test_node_initial_parameters ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_parameter_events_filter test/test_parameter_events_filter.cpp)
  if(TARGET test_parameter_events_filter)
    target_include_directories(test_parameter_events_filter PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_parameter_events_filter ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_parameter test/test_parameter.cpp)
  if(TARGET test_parameter)
    target_include_directories(test_parameter PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_parameter ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_parameter_map test/test_parameter_map.cpp)
  if(TARGET test_parameter_map)
    target_link_libraries(test_parameter_map ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_publisher test/test_publisher.cpp)
  if(TARGET test_publisher)
    target_include_directories(test_publisher PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_publisher ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_rate test/test_rate.cpp)
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
  ament_add_gtest(test_serialized_message_allocator test/test_serialized_message_allocator.cpp)
  if(TARGET test_serialized_message_allocator)
    target_include_directories(test_serialized_message_allocator PUBLIC
      ${test_msgs_INCLUDE_DIRS}
    )
    target_link_libraries(test_serialized_message_allocator
      ${PROJECT_NAME}
      ${test_msgs_LIBRARIES}
    )
  endif()
  ament_add_gtest(test_service test/test_service.cpp)
  if(TARGET test_service)
    target_include_directories(test_service PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_service ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_subscription test/test_subscription.cpp)
  if(TARGET test_subscription)
    target_include_directories(test_subscription PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_generator_cpp_INCLUDE_DIRS}
      ${rosidl_typesupport_cpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_subscription ${PROJECT_NAME})
  endif()
  find_package(test_msgs REQUIRED)
  ament_add_gtest(test_subscription_traits test/test_subscription_traits.cpp)
  if(TARGET test_subscription_traits)
    target_include_directories(test_subscription_traits PUBLIC
      ${rcl_INCLUDE_DIRS}
    )
  ament_target_dependencies(test_subscription_traits
    "test_msgs"
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

  ament_add_gtest(test_duration test/test_duration.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_duration)
    ament_target_dependencies(test_duration
      "rcl")
    target_link_libraries(test_duration ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_executor test/test_executor.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_executor)
    ament_target_dependencies(test_executor
      "rcl")
    target_link_libraries(test_executor ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_logger test/test_logger.cpp)
  target_link_libraries(test_logger ${PROJECT_NAME})

  ament_add_gmock(test_logging test/test_logging.cpp)
  target_link_libraries(test_logging ${PROJECT_NAME})

  ament_add_gtest(test_time test/test_time.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_time)
    ament_target_dependencies(test_time
      "rcl")
    target_link_libraries(test_time ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_time_source test/test_time_source.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_time_source)
    ament_target_dependencies(test_time_source
      "rcl")
    target_link_libraries(test_time_source ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_utilities test/test_utilities.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_utilities)
    ament_target_dependencies(test_utilities
      "rcl")
    target_link_libraries(test_utilities ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_init test/test_init.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_init)
    ament_target_dependencies(test_init
      "rcl")
    target_link_libraries(test_init ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_multi_threaded_executor test/executors/test_multi_threaded_executor.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_multi_threaded_executor)
    ament_target_dependencies(test_multi_threaded_executor
      "rcl")
    target_link_libraries(test_multi_threaded_executor ${PROJECT_NAME})
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
  DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
)
