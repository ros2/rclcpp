find_package(ament_cmake_gtest REQUIRED)

find_package(rmw_implementation_cmake REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

rosidl_generate_interfaces(${PROJECT_NAME}_test_msgs
  ../msg/Header.msg
  ../msg/LargeMessage.msg
  ../msg/MessageWithHeader.msg
  ../msg/String.msg
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
  SKIP_INSTALL
)
# Need the target name to depend on generated interface libraries
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_test_msgs" "rosidl_typesupport_cpp")

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
ament_add_test_label(test_exceptions mimick)
if(TARGET test_exceptions)
  target_link_libraries(test_exceptions ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_allocator_memory_strategy strategies/test_allocator_memory_strategy.cpp)
if(TARGET test_allocator_memory_strategy)
  target_link_libraries(test_allocator_memory_strategy ${PROJECT_NAME} rcpputils::rcpputils ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_message_pool_memory_strategy strategies/test_message_pool_memory_strategy.cpp)
if(TARGET test_message_pool_memory_strategy)
  target_link_libraries(test_message_pool_memory_strategy ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_any_service_callback test_any_service_callback.cpp)
if(TARGET test_any_service_callback)
  target_link_libraries(test_any_service_callback ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_any_subscription_callback test_any_subscription_callback.cpp)
if(TARGET test_any_subscription_callback)
  target_link_libraries(test_any_subscription_callback ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_client test_client.cpp)
ament_add_test_label(test_client mimick)
if(TARGET test_client)
  target_link_libraries(test_client ${PROJECT_NAME} mimick ${rcl_interfaces_TARGETS} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_clock test_clock.cpp)
ament_add_test_label(test_clock mimick)
if(TARGET test_clock)
  target_link_libraries(test_clock ${PROJECT_NAME} mimick ${rcl_interfaces_TARGETS} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_clock_conditional test_clock_conditional.cpp)
ament_add_test_label(test_clock_conditional mimick)
if(TARGET test_clock_conditional)
  target_link_libraries(test_clock_conditional ${PROJECT_NAME} mimick ${rcl_interfaces_TARGETS} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_copy_all_parameter_values test_copy_all_parameter_values.cpp)
if(TARGET test_copy_all_parameter_values)
  target_link_libraries(test_copy_all_parameter_values ${PROJECT_NAME})
endif()
ament_add_gtest(test_create_timer test_create_timer.cpp)
if(TARGET test_create_timer)
  target_link_libraries(test_create_timer ${PROJECT_NAME})
  target_include_directories(test_create_timer PRIVATE ./)
endif()
ament_add_gtest(test_generic_client test_generic_client.cpp)
ament_add_test_label(test_generic_client mimick)
if(TARGET test_generic_client)
  target_link_libraries(test_generic_client ${PROJECT_NAME}
    mimick
    ${rcl_interfaces_TARGETS}
    rmw::rmw
    rosidl_runtime_cpp::rosidl_runtime_cpp
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    ${test_msgs_TARGETS}
  )
endif()
ament_add_gtest(test_generic_service test_generic_service.cpp)
ament_add_test_label(test_generic_service mimick)
if(TARGET test_generic_service)
  target_link_libraries(test_generic_service ${PROJECT_NAME}
    mimick
    ${rcl_interfaces_TARGETS}
    rmw::rmw
    rosidl_runtime_cpp::rosidl_runtime_cpp
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    ${test_msgs_TARGETS}
  )
endif()
ament_add_gtest(test_client_common test_client_common.cpp)
ament_add_test_label(test_client_common mimick)
if(TARGET test_client_common)
  target_link_libraries(test_client_common ${PROJECT_NAME}
    mimick
    ${rcl_interfaces_TARGETS}
    rmw::rmw
    rosidl_runtime_cpp::rosidl_runtime_cpp
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    ${test_msgs_TARGETS}
  )
endif()
ament_add_gtest(test_create_subscription test_create_subscription.cpp)
if(TARGET test_create_subscription)
  target_link_libraries(test_create_subscription ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_expand_topic_or_service_name test_expand_topic_or_service_name.cpp)
ament_add_test_label(test_expand_topic_or_service_name mimick)
if(TARGET test_expand_topic_or_service_name)
  target_link_libraries(test_expand_topic_or_service_name ${PROJECT_NAME} mimick rcl::rcl rmw::rmw)
endif()
ament_add_gtest(test_function_traits test_function_traits.cpp)
if(TARGET test_function_traits)
  target_link_libraries(test_function_traits ${PROJECT_NAME})
endif()
ament_add_gtest(
  test_future_return_code
  test_future_return_code.cpp)
if(TARGET test_future_return_code)
  target_link_libraries(test_future_return_code ${PROJECT_NAME})
endif()
ament_add_gmock(test_intra_process_manager test_intra_process_manager.cpp)
if(TARGET test_intra_process_manager)
  target_link_libraries(test_intra_process_manager ${PROJECT_NAME} ${rcl_interfaces_TARGETS} rmw::rmw)
endif()
ament_add_gmock(test_intra_process_manager_with_allocators test_intra_process_manager_with_allocators.cpp)
if(TARGET test_intra_process_manager_with_allocators)
  target_link_libraries(test_intra_process_manager_with_allocators ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_ring_buffer_implementation test_ring_buffer_implementation.cpp)
if(TARGET test_ring_buffer_implementation)
  target_link_libraries(test_ring_buffer_implementation ${PROJECT_NAME})
endif()
ament_add_gtest(test_intra_process_buffer test_intra_process_buffer.cpp)
if(TARGET test_intra_process_buffer)
  target_link_libraries(test_intra_process_buffer ${PROJECT_NAME})
endif()

ament_add_gtest(test_loaned_message test_loaned_message.cpp)
ament_add_test_label(test_loaned_message mimick)
target_link_libraries(test_loaned_message ${PROJECT_NAME} mimick ${test_msgs_TARGETS})

ament_add_gtest(test_memory_strategy test_memory_strategy.cpp)
target_link_libraries(test_memory_strategy ${PROJECT_NAME} ${test_msgs_TARGETS})

ament_add_gtest(test_message_memory_strategy test_message_memory_strategy.cpp)
target_link_libraries(test_message_memory_strategy ${PROJECT_NAME} ${test_msgs_TARGETS})

ament_add_gtest(test_node test_node.cpp TIMEOUT 240)
ament_add_test_label(test_node mimick)
if(TARGET test_node)
  target_link_libraries(test_node ${PROJECT_NAME} mimick rcpputils::rcpputils rmw::rmw ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_node_interfaces__get_node_interfaces
  node_interfaces/test_get_node_interfaces.cpp)
if(TARGET test_node_interfaces__get_node_interfaces)
  target_link_libraries(test_node_interfaces__get_node_interfaces ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_base
  node_interfaces/test_node_base.cpp)
ament_add_test_label(test_node_interfaces__node_base mimick)
if(TARGET test_node_interfaces__node_base)
  target_link_libraries(test_node_interfaces__node_base ${PROJECT_NAME} mimick rcl::rcl rmw::rmw)
endif()
ament_add_gtest(test_node_interfaces__node_clock
  node_interfaces/test_node_clock.cpp)
if(TARGET test_node_interfaces__node_clock)
  target_link_libraries(test_node_interfaces__node_clock ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_graph
  node_interfaces/test_node_graph.cpp
  TIMEOUT 120)
ament_add_test_label(test_node_interfaces__node_graph mimick)
if(TARGET test_node_interfaces__node_graph)
  target_link_libraries(test_node_interfaces__node_graph ${PROJECT_NAME} mimick rcl::rcl ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_node_interfaces__node_interfaces
  node_interfaces/test_node_interfaces.cpp)
if(TARGET test_node_interfaces__node_interfaces)
  target_link_libraries(test_node_interfaces__node_interfaces ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_parameters
  node_interfaces/test_node_parameters.cpp)
ament_add_test_label(test_node_interfaces__node_parameters mimick)
if(TARGET test_node_interfaces__node_parameters)
  target_link_libraries(test_node_interfaces__node_parameters ${PROJECT_NAME} mimick rcpputils::rcpputils ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_node_interfaces__node_services
  node_interfaces/test_node_services.cpp)
ament_add_test_label(test_node_interfaces__node_services mimick)
if(TARGET test_node_interfaces__node_services)
  target_link_libraries(test_node_interfaces__node_services ${PROJECT_NAME} mimick rcl::rcl)
endif()
ament_add_gtest(test_node_interfaces__node_timers
  node_interfaces/test_node_timers.cpp)
ament_add_test_label(test_node_interfaces__node_timers mimick)
if(TARGET test_node_interfaces__node_timers)
  target_link_libraries(test_node_interfaces__node_timers ${PROJECT_NAME} mimick rcl::rcl)
endif()
ament_add_gtest(test_node_interfaces__node_topics
  node_interfaces/test_node_topics.cpp)
ament_add_test_label(test_node_interfaces__node_topics mimick)
if(TARGET test_node_interfaces__node_topics)
  target_link_libraries(test_node_interfaces__node_topics ${PROJECT_NAME} mimick rcl::rcl ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_node_interfaces__node_type_descriptions
  node_interfaces/test_node_type_descriptions.cpp)
if(TARGET test_node_interfaces__node_type_descriptions)
  target_link_libraries(test_node_interfaces__node_type_descriptions ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_interfaces__node_waitables
  node_interfaces/test_node_waitables.cpp)
ament_add_test_label(test_node_interfaces__node_waitables mimick)
if(TARGET test_node_interfaces__node_waitables)
  target_link_libraries(test_node_interfaces__node_waitables ${PROJECT_NAME} mimick rcl::rcl)
endif()
ament_add_gtest(test_node_interfaces__test_template_utils  # Compile time test
  node_interfaces/detail/test_template_utils.cpp)
if(TARGET test_node_interfaces__test_template_utils)
  target_link_libraries(test_node_interfaces__test_template_utils ${PROJECT_NAME})
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
  target_link_libraries(test_node_global_args ${PROJECT_NAME})
endif()
ament_add_gtest(test_node_options test_node_options.cpp)
ament_add_test_label(test_node_options mimick)
if(TARGET test_node_options)
  target_link_libraries(test_node_options ${PROJECT_NAME} mimick rcl::rcl)
endif()
ament_add_gtest(test_init_options test_init_options.cpp)
ament_add_test_label(test_init_options mimick)
if(TARGET test_init_options)
  target_link_libraries(test_init_options ${PROJECT_NAME} mimick rcl::rcl)
endif()
ament_add_gmock(test_parameter_client test_parameter_client.cpp)
if(TARGET test_parameter_client)
  target_link_libraries(test_parameter_client ${PROJECT_NAME} ${rcl_interfaces_TARGETS})
endif()
ament_add_gtest(test_parameter_service test_parameter_service.cpp)
if(TARGET test_parameter_service)
  target_link_libraries(test_parameter_service ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_events_filter test_parameter_events_filter.cpp)
if(TARGET test_parameter_events_filter)
  target_link_libraries(test_parameter_events_filter ${PROJECT_NAME} ${rcl_interfaces_TARGETS})
endif()
ament_add_gtest(test_parameter test_parameter.cpp)
if(TARGET test_parameter)
  target_link_libraries(test_parameter ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_event_handler test_parameter_event_handler.cpp)
if(TARGET test_parameter_event_handler)
  target_link_libraries(test_parameter_event_handler ${PROJECT_NAME})
endif()
ament_add_gtest(test_parameter_map test_parameter_map.cpp)
if(TARGET test_parameter_map)
  target_link_libraries(test_parameter_map ${PROJECT_NAME} rcl::rcl rcl_yaml_param_parser::rcl_yaml_param_parser rcutils::rcutils)
endif()
ament_add_gtest(test_publisher test_publisher.cpp TIMEOUT 120)
ament_add_test_label(test_publisher mimick)
if(TARGET test_publisher)
  target_link_libraries(test_publisher ${PROJECT_NAME} mimick rcl::rcl rcutils::rcutils ${test_msgs_TARGETS})
endif()

set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
if(WIN32)
  set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
endif()

ament_add_gtest(test_publisher_with_type_adapter test_publisher_with_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_publisher_with_type_adapter)
  target_link_libraries(test_publisher_with_type_adapter
    ${PROJECT_NAME}
    ${cpp_typesupport_target})
endif()

ament_add_gtest(test_subscription_with_type_adapter test_subscription_with_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_with_type_adapter)
  target_link_libraries(test_subscription_with_type_adapter
    ${PROJECT_NAME}
    ${cpp_typesupport_target})
endif()

ament_add_gtest(test_subscription_publisher_with_same_type_adapter test_subscription_publisher_with_same_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_publisher_with_same_type_adapter)
  target_link_libraries(test_subscription_publisher_with_same_type_adapter
    ${PROJECT_NAME}
    ${cpp_typesupport_target}
    ${statistics_msgs_TARGETS}
  )
endif()

ament_add_gtest(test_publisher_subscription_count_api test_publisher_subscription_count_api.cpp)
if(TARGET test_publisher_subscription_count_api)
  target_link_libraries(test_publisher_subscription_count_api ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_qos test_qos.cpp)
if(TARGET test_qos)
  target_link_libraries(test_qos
    ${PROJECT_NAME}
    rmw::rmw
  )
endif()

ament_add_gmock(test_qos_overriding_options test_qos_overriding_options.cpp)
if(TARGET test_qos_overriding_options)
  target_link_libraries(test_qos_overriding_options ${PROJECT_NAME})
endif()
ament_add_gmock(test_qos_parameters test_qos_parameters.cpp)
if(TARGET test_qos_parameters)
  target_link_libraries(test_qos_parameters ${PROJECT_NAME})
endif()
ament_add_gtest(test_rate test_rate.cpp)
if(TARGET test_rate)
  target_link_libraries(test_rate ${PROJECT_NAME})
endif()
ament_add_gtest(test_serialized_message_allocator test_serialized_message_allocator.cpp)
if(TARGET test_serialized_message_allocator)
  target_link_libraries(test_serialized_message_allocator ${PROJECT_NAME} rcl::rcl ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_serialized_message test_serialized_message.cpp)
if(TARGET test_serialized_message)
  target_link_libraries(test_serialized_message ${PROJECT_NAME} rcpputils::rcpputils ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_service test_service.cpp)
ament_add_test_label(test_service mimick)
if(TARGET test_service)
  target_link_libraries(test_service ${PROJECT_NAME} mimick ${rcl_interfaces_TARGES} ${test_msgs_TARGETS})
endif()
ament_add_gmock(test_service_introspection test_service_introspection.cpp)
ament_add_test_label(test_service_introspection mimick)
if(TARGET test_service_introspection)
  target_link_libraries(test_service_introspection ${PROJECT_NAME} mimick ${service_msgs_TARGETS} ${test_msgs_TARGETS})
endif()
# Creating and destroying nodes is slow with Connext, so this needs larger timeout.
ament_add_gtest(test_subscription test_subscription.cpp TIMEOUT 120)
ament_add_test_label(test_subscription mimick)
if(TARGET test_subscription)
  target_link_libraries(test_subscription ${PROJECT_NAME} mimick ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_subscription_publisher_count_api test_subscription_publisher_count_api.cpp)
if(TARGET test_subscription_publisher_count_api)
  target_link_libraries(test_subscription_publisher_count_api ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_subscription_traits test_subscription_traits.cpp)
if(TARGET test_subscription_traits)
  target_link_libraries(test_subscription_traits ${PROJECT_NAME} rcl::rcl ${test_msgs_TARGETS})
endif()
ament_add_gtest(test_type_support test_type_support.cpp)
if(TARGET test_type_support)
  target_link_libraries(test_type_support ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()
ament_add_gmock(test_typesupport_helpers test_typesupport_helpers.cpp)
if(TARGET test_typesupport_helpers)
  target_link_libraries(test_typesupport_helpers ${PROJECT_NAME} rcpputils::rcpputils)
endif()
ament_add_gtest(test_find_weak_nodes test_find_weak_nodes.cpp)
if(TARGET test_find_weak_nodes)
  target_link_libraries(test_find_weak_nodes ${PROJECT_NAME})
endif()

ament_add_gtest(test_externally_defined_services test_externally_defined_services.cpp)
target_link_libraries(test_externally_defined_services ${PROJECT_NAME} rcl::rcl ${test_msgs_TARGETS})

ament_add_gtest(test_duration test_duration.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_duration)
  target_link_libraries(test_duration ${PROJECT_NAME} rcl::rcl)
endif()

ament_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger ${PROJECT_NAME} rcutils::rcutils)

ament_add_gmock(test_logging test_logging.cpp)
target_link_libraries(test_logging ${PROJECT_NAME} rcutils::rcutils)

ament_add_gmock(test_context test_context.cpp)
target_link_libraries(test_context ${PROJECT_NAME})

ament_add_gtest(test_time test_time.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_time)
  target_link_libraries(test_time ${PROJECT_NAME} rcl::rcl rcutils::rcutils)
endif()

ament_add_gtest(test_timer test_timer.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
ament_add_test_label(test_timer mimick)
if(TARGET test_timer)
  target_link_libraries(test_timer ${PROJECT_NAME} mimick rcl::rcl)
endif()

ament_add_gtest(test_timers_manager test_timers_manager.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_timers_manager)
  target_link_libraries(test_timers_manager ${PROJECT_NAME})
endif()

ament_add_gtest(test_time_source test_time_source.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_time_source)
  target_link_libraries(test_time_source ${PROJECT_NAME} rcl::rcl)
endif()

ament_add_gtest(test_utilities test_utilities.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
ament_add_test_label(test_utilities mimick)
if(TARGET test_utilities)
  target_link_libraries(test_utilities ${PROJECT_NAME} mimick rcl::rcl)
endif()

ament_add_gtest(test_wait_for_message test_wait_for_message.cpp)
if(TARGET test_wait_for_message)
  target_link_libraries(test_wait_for_message ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_logger_service test_logger_service.cpp)
if(TARGET test_logger_service)
  target_link_libraries(test_logger_service ${PROJECT_NAME} ${rcl_interfaces_TARGETS})
endif()

ament_add_gtest(test_interface_traits test_interface_traits.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_interface_traits)
  target_link_libraries(test_interface_traits ${PROJECT_NAME})
endif()

ament_add_gtest(test_reinitialized_timers
  executors/test_reinitialized_timers.cpp
  TIMEOUT 30)
if(TARGET test_reinitialized_timers)
  target_link_libraries(test_reinitialized_timers ${PROJECT_NAME})
endif()

ament_add_gtest(
  test_executors
  executors/test_executors.cpp
  executors/test_waitable.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND MSVC)
  target_compile_options(test_executors PRIVATE "/bigobj")
endif()
if(TARGET test_executors)
  target_link_libraries(test_executors ${PROJECT_NAME} rcl::rcl ${test_msgs_TARGETS})
endif()

ament_add_gtest(
  test_executors_timer_cancel_behavior
  executors/test_executors_timer_cancel_behavior.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors_timer_cancel_behavior)
  target_link_libraries(test_executors_timer_cancel_behavior ${PROJECT_NAME} ${rosgraph_msgs_TARGETS})
endif()

ament_add_gtest(
  test_executors_callback_group_behavior
  executors/test_executors_callback_group_behavior.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors_callback_group_behavior)
  target_link_libraries(test_executors_callback_group_behavior ${PROJECT_NAME})
endif()

ament_add_gtest(
  test_executors_intraprocess
  executors/test_executors_intraprocess.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors_intraprocess)
  target_link_libraries(test_executors_intraprocess ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(
  test_executors_busy_waiting
  executors/test_executors_busy_waiting.cpp
  executors/test_waitable.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors_busy_waiting)
  target_link_libraries(test_executors_busy_waiting ${PROJECT_NAME})
endif()

ament_add_gtest(
  test_executors_warmup
  executors/test_executors_warmup.cpp
  executors/test_waitable.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 180)
if(TARGET test_executors_warmup)
  target_link_libraries(test_executors_warmup ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_static_single_threaded_executor executors/test_static_single_threaded_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
ament_add_test_label(test_static_single_threaded_executor mimick)
if(TARGET test_static_single_threaded_executor)
  target_link_libraries(test_static_single_threaded_executor ${PROJECT_NAME} mimick ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_multi_threaded_executor executors/test_multi_threaded_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_multi_threaded_executor)
  target_link_libraries(test_multi_threaded_executor ${PROJECT_NAME})
endif()

ament_add_gtest(test_entities_collector executors/test_entities_collector.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}" TIMEOUT 120)
if(TARGET test_entities_collector)
  target_link_libraries(test_entities_collector ${PROJECT_NAME})
endif()

ament_add_gtest(test_executor_notify_waitable executors/test_executor_notify_waitable.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}" TIMEOUT 120)
ament_add_test_label(test_executor_notify_waitable mimick)
if(TARGET test_executor_notify_waitable)
  target_link_libraries(test_executor_notify_waitable ${PROJECT_NAME} mimick rcpputils::rcpputils)
endif()

ament_add_gtest(test_events_executor executors/test_events_executor.cpp TIMEOUT 60)
if(TARGET test_events_executor)
  target_link_libraries(test_events_executor ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_events_queue executors/test_events_queue.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_events_queue)
  target_link_libraries(test_events_queue ${PROJECT_NAME})
endif()

ament_add_gtest(test_guard_condition test_guard_condition.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
ament_add_test_label(test_guard_condition mimick)
if(TARGET test_guard_condition)
  target_link_libraries(test_guard_condition ${PROJECT_NAME} mimick)
endif()

ament_add_gtest(test_wait_set test_wait_set.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}")
if(TARGET test_wait_set)
  target_link_libraries(test_wait_set ${PROJECT_NAME} ${rcl_interfaces_TARGETS} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_subscription_topic_statistics topic_statistics/test_subscription_topic_statistics.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_topic_statistics)
  target_link_libraries(test_subscription_topic_statistics
    ${PROJECT_NAME}
    libstatistics_collector::libstatistics_collector
    ${statistics_msgs_TARGETS}
    ${test_msgs_TARGETS}
  )
endif()

ament_add_gtest(test_subscription_options test_subscription_options.cpp)
if(TARGET test_subscription_options)
  target_link_libraries(test_subscription_options ${PROJECT_NAME})
endif()

ament_add_gtest(test_dynamic_storage wait_set_policies/test_dynamic_storage.cpp)
if(TARGET test_dynamic_storage)
  target_link_libraries(test_dynamic_storage ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_storage_policy_common wait_set_policies/test_storage_policy_common.cpp)
ament_add_test_label(test_storage_policy_common mimick)
if(TARGET test_storage_policy_common)
  target_link_libraries(test_storage_policy_common ${PROJECT_NAME} mimick ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_static_storage wait_set_policies/test_static_storage.cpp)
if(TARGET test_static_storage)
  target_link_libraries(test_static_storage ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_thread_safe_synchronization wait_set_policies/test_thread_safe_synchronization.cpp)
if(TARGET test_thread_safe_synchronization)
  target_link_libraries(test_thread_safe_synchronization ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_intra_process_waitable waitables/test_intra_process_waitable.cpp)
if(TARGET test_intra_process_waitable)
  target_link_libraries(test_intra_process_waitable ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_rosout_qos test_rosout_qos.cpp)
if(TARGET test_rosout_qos)
  target_link_libraries(test_rosout_qos ${PROJECT_NAME} rcl::rcl rmw::rmw)
endif()

ament_add_gtest(test_rosout_subscription test_rosout_subscription.cpp)
if(TARGET test_rosout_subscription)
  target_link_libraries(test_rosout_subscription ${PROJECT_NAME} ${rcl_interfaces_TARGETS})
endif()

ament_add_gtest(test_executor test_executor.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
  TIMEOUT 120)
ament_add_test_label(test_executor mimick)
if(TARGET test_executor)
  target_link_libraries(test_executor ${PROJECT_NAME} mimick ${test_msgs_TARGETS})
endif()

ament_add_gtest(test_graph_listener test_graph_listener.cpp)
ament_add_test_label(test_graph_listener mimick)
if(TARGET test_graph_listener)
  target_link_libraries(test_graph_listener ${PROJECT_NAME} mimick)
endif()

ament_add_gmock_executable(test_qos_event test_qos_event.cpp)
if(TARGET test_qos_event)
  target_link_libraries(test_qos_event ${PROJECT_NAME} mimick rcutils::rcutils rmw::rmw ${test_msgs_TARGETS})
endif()

ament_add_gmock_executable(test_generic_pubsub test_generic_pubsub.cpp)
if(TARGET test_generic_pubsub)
  target_link_libraries(test_generic_pubsub ${PROJECT_NAME} rcl::rcl ${test_msgs_TARGETS})
endif()

ament_add_gmock_executable(test_add_callback_groups_to_executor test_add_callback_groups_to_executor.cpp)
if(TARGET test_add_callback_groups_to_executor)
  target_link_libraries(test_add_callback_groups_to_executor ${PROJECT_NAME} ${test_msgs_TARGETS})
endif()

ament_add_gmock_executable(test_subscription_content_filter test_subscription_content_filter.cpp)
if(TARGET test_subscription_content_filter)
  target_link_libraries(test_subscription_content_filter ${PROJECT_NAME} mimick ${test_msgs_TARGETS})
endif()

function(test_on_all_rmws)
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

  # If the rmw_implementation is rmw_zenoh_cpp, run the tests with multicast discovery enabled.
  # Note: This is a temporary change that will be reverted before we branch into Kilted.
  if(rmw_implementation STREQUAL "rmw_zenoh_cpp")
    list(APPEND rmw_implementation_env_var
      ZENOH_CONFIG_OVERRIDE=scouting/multicast/enabled=true
    )
  endif()

  ament_add_gmock_test(test_qos_event
    TEST_NAME test_qos_event${target_suffix}
    ENV ${rmw_implementation_env_var}
  )
  ament_add_test_label(test_qos_event${target_suffix} mimick)

  ament_add_gmock_test(test_generic_pubsub
    TEST_NAME test_generic_pubsub${target_suffix}
    ENV ${rmw_implementation_env_var}
  )

  ament_add_gmock_test(test_add_callback_groups_to_executor
    TEST_NAME test_add_callback_groups_to_executor${target_suffix}
    ENV ${rmw_implementation_env_var}
    TIMEOUT 120
  )

  ament_add_gmock_test(test_subscription_content_filter
    TEST_NAME test_subscription_content_filter${target_suffix}
    ENV ${rmw_implementation_env_var}
    TIMEOUT 120
  )
  ament_add_test_label(test_subscription_content_filter${target_suffix} mimick)
endfunction()
call_for_each_rmw_implementation(test_on_all_rmws)
