find_package(ament_cmake_gtest REQUIRED)

find_package(rmw_implementation_cmake REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

rosidl_generate_interfaces(${PROJECT_NAME}_test_msgs
  ../msg/Header.msg
  ../msg/MessageWithHeader.msg
  ../msg/String.msg
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
  SKIP_INSTALL
)

ament_add_gtest(
  test_allocator_common
  allocator/test_allocator_common.cpp)
if(TARGET test_allocator_common)
  target_link_libraries(test_allocator_common ${PROJECT_NAME})
endif()
ament_add_gtest(
  test_allocator_deleter
  allocator/test_allocator_deleter.cpp)
if(TARGET test_allocator_deleter)
  target_link_libraries(test_allocator_deleter ${PROJECT_NAME})
endif()
ament_add_gtest(
  test_exceptions
  exceptions/test_exceptions.cpp)
if(TARGET test_exceptions)
  target_link_libraries(test_exceptions ${PROJECT_NAME} mimick)
endif()

# Increasing timeout because connext can take a long time to destroy nodes
# TODO(brawner) remove when destroying Node for Connext is resolved. See:
# https://github.com/ros2/rclcpp/issues/1250
ament_add_gtest(
  test_allocator_memory_strategy
  strategies/test_allocator_memory_strategy.cpp
  TIMEOUT 360)
if(TARGET test_allocator_memory_strategy)
  ament_target_dependencies(test_allocator_memory_strategy
    "rcl"
    "test_msgs"
  )
  target_link_libraries(test_allocator_memory_strategy ${PROJECT_NAME})
endif()
ament_add_gtest(test_message_pool_memory_strategy strategies/test_message_pool_memory_strategy.cpp)
if(TARGET test_message_pool_memory_strategy)
  ament_target_dependencies(test_message_pool_memory_strategy
    "rcl"
    "test_msgs"
  )
  target_link_libraries(test_message_pool_memory_strategy ${PROJECT_NAME})
endif()
ament_add_gtest(test_any_service_callback test_any_service_callback.cpp)
if(TARGET test_any_service_callback)
  ament_target_dependencies(test_any_service_callback
    "test_msgs"
  )
  target_link_libraries(test_any_service_callback ${PROJECT_NAME})
endif()
ament_add_gtest(test_any_subscription_callback test_any_subscription_callback.cpp)
if(TARGET test_any_subscription_callback)
  ament_target_dependencies(test_any_subscription_callback
    "test_msgs"
  )
  target_link_libraries(test_any_subscription_callback ${PROJECT_NAME})
endif()
ament_add_gtest(test_client test_client.cpp)
if(TARGET test_client)
  ament_target_dependencies(test_client
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_client ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_create_timer test_create_timer.cpp)
if(TARGET test_create_timer)
  ament_target_dependencies(test_create_timer
    "rcl_interfaces"
    "rmw"
    "rcl"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_create_timer ${PROJECT_NAME})
  target_include_directories(test_create_timer PRIVATE ./)
endif()
ament_add_gtest(test_create_subscription test_create_subscription.cpp)
if(TARGET test_create_subscription)
  target_link_libraries(test_create_subscription ${PROJECT_NAME})
  ament_target_dependencies(test_create_subscription
    "test_msgs"
  )
endif()
ament_add_gtest(test_add_callback_groups_to_executor
  test_add_callback_groups_to_executor.cpp
  TIMEOUT 120)
if(TARGET test_add_callback_groups_to_executor)
  target_link_libraries(test_add_callback_groups_to_executor ${PROJECT_NAME})
  ament_target_dependencies(test_add_callback_groups_to_executor
    "test_msgs"
  )
endif()
ament_add_gtest(test_expand_topic_or_service_name test_expand_topic_or_service_name.cpp)
if(TARGET test_expand_topic_or_service_name)
  ament_target_dependencies(test_expand_topic_or_service_name
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_expand_topic_or_service_name ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_function_traits test_function_traits.cpp)
if(TARGET test_function_traits)
  target_include_directories(test_function_traits PUBLIC ../../include)
  ament_target_dependencies(test_function_traits
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
endif()
ament_add_gtest(
  test_future_return_code
  test_future_return_code.cpp)
if(TARGET test_future_return_code)
  target_link_libraries(test_future_return_code ${PROJECT_NAME})
endif()
ament_add_gmock(test_intra_process_manager test_intra_process_manager.cpp)
if(TARGET test_intra_process_manager)
  ament_target_dependencies(test_intra_process_manager
    "rcl"
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_intra_process_manager ${PROJECT_NAME})
endif()
ament_add_gmock(test_intra_process_manager_with_allocators test_intra_process_manager_with_allocators.cpp)
if(TARGET test_intra_process_manager_with_allocators)
  ament_target_dependencies(test_intra_process_manager_with_allocators
    "test_msgs"
  )
  target_link_libraries(test_intra_process_manager_with_allocators ${PROJECT_NAME})
endif()
ament_add_gtest(test_ring_buffer_implementation test_ring_buffer_implementation.cpp)
if(TARGET test_ring_buffer_implementation)
  ament_target_dependencies(test_ring_buffer_implementation
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_ring_buffer_implementation ${PROJECT_NAME})
endif()
ament_add_gtest(test_intra_process_buffer test_intra_process_buffer.cpp)
if(TARGET test_intra_process_buffer)
  ament_target_dependencies(test_intra_process_buffer
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_intra_process_buffer ${PROJECT_NAME})
endif()

ament_add_gtest(test_loaned_message test_loaned_message.cpp)
ament_target_dependencies(test_loaned_message
  "test_msgs"
)
target_link_libraries(test_loaned_message ${PROJECT_NAME} mimick)

ament_add_gtest(test_memory_strategy test_memory_strategy.cpp)
ament_target_dependencies(test_memory_strategy
  "test_msgs"
)
target_link_libraries(test_memory_strategy ${PROJECT_NAME})

ament_add_gtest(test_message_memory_strategy test_message_memory_strategy.cpp)
ament_target_dependencies(test_message_memory_strategy
  "test_msgs"
)
target_link_libraries(test_message_memory_strategy ${PROJECT_NAME})

ament_add_gtest(test_node test_node.cpp TIMEOUT 240)
if(TARGET test_node)
  ament_target_dependencies(test_node
    "rcl_interfaces"
    "rcpputils"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_node ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_node_interfaces__get_node_interfaces
  node_interfaces/test_get_node_interfaces.cpp)
if(TARGET test_node_interfaces__get_node_interfaces)
  target_link_libraries(test_node_interfaces__get_node_interfaces ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_base
  node_interfaces/test_node_base.cpp)
if(TARGET test_node_interfaces__node_base)
  target_link_libraries(test_node_interfaces__node_base ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_clock
  node_interfaces/test_node_clock.cpp)
if(TARGET test_node_interfaces__node_clock)
  target_link_libraries(test_node_interfaces__node_clock ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_graph
  node_interfaces/test_node_graph.cpp
  TIMEOUT 120)
if(TARGET test_node_interfaces__node_graph)
  ament_target_dependencies(
    test_node_interfaces__node_graph
    "test_msgs")
  target_link_libraries(test_node_interfaces__node_graph ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_parameters
  node_interfaces/test_node_parameters.cpp)
if(TARGET test_node_interfaces__node_parameters)
  target_link_libraries(test_node_interfaces__node_parameters ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_services
  node_interfaces/test_node_services.cpp)
if(TARGET test_node_interfaces__node_services)
  target_link_libraries(test_node_interfaces__node_services ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_timers
  node_interfaces/test_node_timers.cpp)
if(TARGET test_node_interfaces__node_timers)
  target_link_libraries(test_node_interfaces__node_timers ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_topics
  node_interfaces/test_node_topics.cpp)
if(TARGET test_node_interfaces__node_topics)
  ament_target_dependencies(
    test_node_interfaces__node_topics
    "test_msgs")
  target_link_libraries(test_node_interfaces__node_topics ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_node_interfaces__node_waitables
  node_interfaces/test_node_waitables.cpp)
if(TARGET test_node_interfaces__node_waitables)
  target_link_libraries(test_node_interfaces__node_waitables ${PROJECT_NAME} mimick)
endif()

# TODO(wjwwood): reenable these build failure tests when I can get Jenkins to ignore their output
# rclcpp_add_build_failure_test(build_failure__get_node_topics_interface_const_ref_rclcpp_node
#   node_interfaces/test_does_not_compile/get_node_topics_interface_const_ref_rclcpp_node.cpp)
# target_link_libraries(build_failure__get_node_topics_interface_const_ref_rclcpp_node
#   ${PROJECT_NAME})

# rclcpp_add_build_failure_test(build_failure__get_node_topics_interface_const_ptr_rclcpp_node
#   node_interfaces/test_does_not_compile/get_node_topics_interface_const_ptr_rclcpp_node.cpp)
# target_link_libraries(build_failure__get_node_topics_interface_const_ptr_rclcpp_node
#   ${PROJECT_NAME})

# rclcpp_add_build_failure_test(build_failure__get_node_topics_interface_const_ref_wrapped_node
#   node_interfaces/test_does_not_compile/get_node_topics_interface_const_ref_wrapped_node.cpp)
# target_link_libraries(build_failure__get_node_topics_interface_const_ref_rclcpp_node
#   ${PROJECT_NAME})

# rclcpp_add_build_failure_test(build_failure__get_node_topics_interface_const_ptr_wrapped_node
#   node_interfaces/test_does_not_compile/get_node_topics_interface_const_ptr_wrapped_node.cpp)
# target_link_libraries(build_failure__get_node_topics_interface_const_ptr_rclcpp_node
#   ${PROJECT_NAME})

ament_add_gtest(test_node_global_args test_node_global_args.cpp)
if(TARGET test_node_global_args)
  ament_target_dependencies(test_node_global_args
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_node_global_args ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_options test_node_options.cpp)
if(TARGET test_node_options)
  ament_target_dependencies(test_node_options "rcl")
  target_link_libraries(test_node_options ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_init_options test_init_options.cpp)
if(TARGET test_init_options)
  ament_target_dependencies(test_init_options "rcl")
  target_link_libraries(test_init_options ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_parameter_client test_parameter_client.cpp)
if(TARGET test_parameter_client)
  ament_target_dependencies(test_parameter_client
    "rcl_interfaces"
  )
  target_link_libraries(test_parameter_client ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_service test_parameter_service.cpp)
if(TARGET test_parameter_service)
  ament_target_dependencies(test_parameter_service
    "rcl_interfaces"
  )
  target_link_libraries(test_parameter_service ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_events_filter test_parameter_events_filter.cpp)
if(TARGET test_parameter_events_filter)
  ament_target_dependencies(test_parameter_events_filter
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_parameter_events_filter ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter test_parameter.cpp)
if(TARGET test_parameter)
  ament_target_dependencies(test_parameter
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_parameter ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_event_handler test_parameter_event_handler.cpp)
if(TARGET test_parameter_event_handler)
  ament_target_dependencies(test_parameter_event_handler
    "rcl_interfaces"
    "rmw"
    "rosidl_generator_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_parameter_event_handler ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_map test_parameter_map.cpp)
if(TARGET test_parameter_map)
  target_link_libraries(test_parameter_map ${PROJECT_NAME})
endif()
ament_add_gtest(test_publisher test_publisher.cpp TIMEOUT 120)
if(TARGET test_publisher)
  ament_target_dependencies(test_publisher
    "rcl"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_publisher ${PROJECT_NAME} mimick)
endif()

set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
if(WIN32)
  set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
endif()

ament_add_gtest(test_publisher_with_type_adapter test_publisher_with_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_publisher_with_type_adapter)
  ament_target_dependencies(test_publisher_with_type_adapter
    "rcutils"
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  rosidl_target_interfaces(test_publisher_with_type_adapter ${PROJECT_NAME}_test_msgs "rosidl_typesupport_cpp")
  target_link_libraries(test_publisher_with_type_adapter ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_subscription_with_type_adapter test_subscription_with_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_with_type_adapter)
  ament_target_dependencies(test_subscription_with_type_adapter
    "rcutils"
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  rosidl_target_interfaces(test_subscription_with_type_adapter ${PROJECT_NAME}_test_msgs "rosidl_typesupport_cpp")
  target_link_libraries(test_subscription_with_type_adapter ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_publisher_subscription_count_api test_publisher_subscription_count_api.cpp)
if(TARGET test_publisher_subscription_count_api)
  ament_target_dependencies(test_publisher_subscription_count_api
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_publisher_subscription_count_api ${PROJECT_NAME})
endif()
ament_add_gtest(test_qos test_qos.cpp)
if(TARGET test_qos)
  ament_target_dependencies(test_qos
    "rmw"
  )
  target_link_libraries(test_qos
    ${PROJECT_NAME}
  )
endif()
function(test_generic_pubsub_for_rmw_implementation)
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})
  ament_add_gmock(test_generic_pubsub${target_suffix} test_generic_pubsub.cpp
    ENV ${rmw_implementation_env_var}
  )
  if(TARGET test_generic_pubsub${target_suffix})
    target_link_libraries(test_generic_pubsub${target_suffix} ${PROJECT_NAME})
    ament_target_dependencies(test_generic_pubsub${target_suffix}
      "rcpputils"
      "rosidl_typesupport_cpp"
      "test_msgs"
    )
  endif()
endfunction()
call_for_each_rmw_implementation(test_generic_pubsub_for_rmw_implementation)
ament_add_gtest(test_qos_event test_qos_event.cpp)
if(TARGET test_qos_event)
  ament_target_dependencies(test_qos_event
    "rmw"
    "test_msgs"
  )
  target_link_libraries(test_qos_event
    ${PROJECT_NAME}
    mimick
  )
endif()
ament_add_gmock(test_qos_overriding_options test_qos_overriding_options.cpp)
if(TARGET test_qos_overriding_options)
  target_link_libraries(test_qos_overriding_options
    ${PROJECT_NAME}
  )
endif()
ament_add_gmock(test_qos_parameters test_qos_parameters.cpp)
if(TARGET test_qos_parameters)
  target_link_libraries(test_qos_parameters
    ${PROJECT_NAME}
  )
endif()
ament_add_gtest(test_rate test_rate.cpp)
if(TARGET test_rate)
  ament_target_dependencies(test_rate
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_rate
    ${PROJECT_NAME}
  )
endif()
ament_add_gtest(test_serialized_message_allocator test_serialized_message_allocator.cpp)
if(TARGET test_serialized_message_allocator)
  ament_target_dependencies(test_serialized_message_allocator
    test_msgs
  )
  target_link_libraries(test_serialized_message_allocator
    ${PROJECT_NAME}
  )
endif()
ament_add_gtest(test_serialized_message test_serialized_message.cpp)
if(TARGET test_serialized_message)
  ament_target_dependencies(test_serialized_message
    test_msgs
  )
  target_link_libraries(test_serialized_message
    ${PROJECT_NAME}
  )
endif()
ament_add_gtest(test_service test_service.cpp)
if(TARGET test_service)
  ament_target_dependencies(test_service
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_service ${PROJECT_NAME} mimick)
endif()
# Creating and destroying nodes is slow with Connext, so this needs larger timeout.
ament_add_gtest(test_subscription test_subscription.cpp TIMEOUT 120)
if(TARGET test_subscription)
  ament_target_dependencies(test_subscription
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_subscription ${PROJECT_NAME} mimick)
endif()
ament_add_gtest(test_subscription_publisher_count_api test_subscription_publisher_count_api.cpp)
if(TARGET test_subscription_publisher_count_api)
  ament_target_dependencies(test_subscription_publisher_count_api
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_subscription_publisher_count_api ${PROJECT_NAME})
endif()
ament_add_gtest(test_subscription_traits test_subscription_traits.cpp)
if(TARGET test_subscription_traits)
  ament_target_dependencies(test_subscription_traits
    "rcl"
    "test_msgs"
  )
  target_link_libraries(test_subscription_traits ${PROJECT_NAME})
endif()
ament_add_gtest(test_type_support test_type_support.cpp)
if(TARGET test_type_support)
  ament_target_dependencies(test_type_support
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_type_support ${PROJECT_NAME})
endif()
ament_add_gmock(test_typesupport_helpers test_typesupport_helpers.cpp)
if(TARGET test_typesupport_helpers)
  target_link_libraries(test_typesupport_helpers ${PROJECT_NAME})
endif()
ament_add_gtest(test_find_weak_nodes test_find_weak_nodes.cpp)
if(TARGET test_find_weak_nodes)
  ament_target_dependencies(test_find_weak_nodes
    "rcl"
  )
  target_link_libraries(test_find_weak_nodes ${PROJECT_NAME})
endif()

ament_add_gtest(test_externally_defined_services test_externally_defined_services.cpp)
ament_target_dependencies(test_externally_defined_services
  "rcl"
  "test_msgs"
)
target_link_libraries(test_externally_defined_services ${PROJECT_NAME})

ament_add_gtest(test_duration test_duration.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_duration)
  ament_target_dependencies(test_duration
    "rcl")
  target_link_libraries(test_duration ${PROJECT_NAME})
endif()

ament_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger ${PROJECT_NAME})

ament_add_gmock(test_logging test_logging.cpp)
target_link_libraries(test_logging ${PROJECT_NAME})

ament_add_gtest(test_time test_time.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_time)
  ament_target_dependencies(test_time
    "rcl")
  target_link_libraries(test_time ${PROJECT_NAME})
endif()

ament_add_gtest(test_timer test_timer.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_timer)
  ament_target_dependencies(test_timer
    "rcl")
  target_link_libraries(test_timer ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_time_source test_time_source.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_time_source)
  ament_target_dependencies(test_time_source
    "rcl")
  target_link_libraries(test_time_source ${PROJECT_NAME})
endif()

ament_add_gtest(test_utilities test_utilities.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_utilities)
  ament_target_dependencies(test_utilities
    "rcl")
  target_link_libraries(test_utilities ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_wait_for_message test_wait_for_message.cpp)
if(TARGET test_wait_for_message)
  ament_target_dependencies(test_wait_for_message
    "test_msgs")
  target_link_libraries(test_wait_for_message ${PROJECT_NAME})
endif()

ament_add_gtest(test_interface_traits test_interface_traits.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_interface_traits)
  ament_target_dependencies(test_interface_traits
    "rcl")
  target_link_libraries(test_interface_traits ${PROJECT_NAME})
endif()

# TODO(brawner) remove when destroying Node for Connext is resolved. See:
# https://github.com/ros2/rclcpp/issues/1250
ament_add_gtest(
  test_executors
  executors/test_executors.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors)
  ament_target_dependencies(test_executors
    "rcl"
    "test_msgs")
  target_link_libraries(test_executors ${PROJECT_NAME})
endif()

ament_add_gtest(test_static_single_threaded_executor executors/test_static_single_threaded_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_static_single_threaded_executor)
  ament_target_dependencies(test_static_single_threaded_executor
    "test_msgs")
  target_link_libraries(test_static_single_threaded_executor ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_multi_threaded_executor executors/test_multi_threaded_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_multi_threaded_executor)
  ament_target_dependencies(test_multi_threaded_executor
    "rcl")
  target_link_libraries(test_multi_threaded_executor ${PROJECT_NAME})
endif()

ament_add_gtest(test_static_executor_entities_collector executors/test_static_executor_entities_collector.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}" TIMEOUT 120)
if(TARGET test_static_executor_entities_collector)
  ament_target_dependencies(test_static_executor_entities_collector
    "rcl"
    "test_msgs")
  target_link_libraries(test_static_executor_entities_collector ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_guard_condition test_guard_condition.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_guard_condition)
  target_link_libraries(test_guard_condition ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_wait_set test_wait_set.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_wait_set)
  ament_target_dependencies(test_wait_set "test_msgs")
  target_link_libraries(test_wait_set ${PROJECT_NAME})
endif()

ament_add_gtest(test_subscription_topic_statistics topic_statistics/test_subscription_topic_statistics.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_topic_statistics)
  ament_target_dependencies(test_subscription_topic_statistics
    "builtin_interfaces"
    "libstatistics_collector"
    "rcl_interfaces"
    "rcutils"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "statistics_msgs"
    "test_msgs")
  rosidl_target_interfaces(test_subscription_topic_statistics ${PROJECT_NAME}_test_msgs "rosidl_typesupport_cpp")
  target_link_libraries(test_subscription_topic_statistics ${PROJECT_NAME})
endif()

ament_add_gtest(test_subscription_options test_subscription_options.cpp)
if(TARGET test_subscription_options)
  ament_target_dependencies(test_subscription_options "rcl")
  target_link_libraries(test_subscription_options ${PROJECT_NAME})
endif()

ament_add_gtest(test_dynamic_storage wait_set_policies/test_dynamic_storage.cpp)
if(TARGET test_dynamic_storage)
  ament_target_dependencies(test_dynamic_storage "rcl" "test_msgs")
  target_link_libraries(test_dynamic_storage ${PROJECT_NAME})
endif()

ament_add_gtest(test_storage_policy_common wait_set_policies/test_storage_policy_common.cpp)
if(TARGET test_storage_policy_common)
  ament_target_dependencies(test_storage_policy_common "rcl" "test_msgs")
  target_link_libraries(test_storage_policy_common ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_static_storage wait_set_policies/test_static_storage.cpp)
if(TARGET test_static_storage)
  ament_target_dependencies(test_static_storage "rcl" "test_msgs")
  target_link_libraries(test_static_storage ${PROJECT_NAME})
endif()

ament_add_gtest(test_thread_safe_synchronization wait_set_policies/test_thread_safe_synchronization.cpp)
if(TARGET test_thread_safe_synchronization)
  ament_target_dependencies(test_thread_safe_synchronization "rcl" "test_msgs")
  target_link_libraries(test_thread_safe_synchronization ${PROJECT_NAME})
endif()

ament_add_gtest(test_rosout_qos test_rosout_qos.cpp)
if(TARGET test_rosout_qos)
  ament_target_dependencies(test_rosout_qos "rcl")
  target_link_libraries(test_rosout_qos ${PROJECT_NAME})
endif()

ament_add_gtest(test_executor test_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 120)
if(TARGET test_executor)
  ament_target_dependencies(test_executor "rcl")
  target_link_libraries(test_executor ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_graph_listener test_graph_listener.cpp)
if(TARGET test_graph_listener)
  target_link_libraries(test_graph_listener ${PROJECT_NAME} mimick)
endif()
