find_package(ament_cmake_google_benchmark REQUIRED)
find_package(performance_test_fixture REQUIRED)

# These benchmarks are only being created and run for the default RMW
# implementation. We are looking to test the performance of the ROS 2 code, not
# the underlying middleware.

add_performance_test(benchmark_client benchmark_client.cpp)
if(TARGET benchmark_client)
  target_link_libraries(benchmark_client ${PROJECT_NAME})
  ament_target_dependencies(benchmark_client test_msgs rcl_interfaces)
endif()

add_performance_test(benchmark_executor benchmark_executor.cpp)
if(TARGET benchmark_executor)
  target_link_libraries(benchmark_executor ${PROJECT_NAME})
  ament_target_dependencies(benchmark_executor test_msgs)
endif()

add_performance_test(benchmark_init_shutdown benchmark_init_shutdown.cpp)
if(TARGET benchmark_init_shutdown)
  target_link_libraries(benchmark_init_shutdown ${PROJECT_NAME})
endif()

add_performance_test(benchmark_node benchmark_node.cpp)
if(TARGET benchmark_node)
  target_link_libraries(benchmark_node ${PROJECT_NAME})
endif()

add_performance_test(benchmark_node_parameters_interface benchmark_node_parameters_interface.cpp)
if(TARGET benchmark_node_parameters_interface)
  target_link_libraries(benchmark_node_parameters_interface ${PROJECT_NAME})
endif()

ament_add_google_benchmark(benchmark_parameter_client benchmark_parameter_client.cpp)
if(TARGET benchmark_parameter_client)
  target_link_libraries(benchmark_parameter_client ${PROJECT_NAME})
endif()

add_performance_test(benchmark_service benchmark_service.cpp)
if(TARGET benchmark_service)
  target_link_libraries(benchmark_service ${PROJECT_NAME})
  ament_target_dependencies(benchmark_service test_msgs rcl_interfaces)
endif()
