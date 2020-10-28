find_package(performance_test_fixture REQUIRED)

# These benchmarks are only being created and run for the default RMW
# implementation. We are looking to test the performance of the ROS 2 code, not
# the underlying middleware.

add_performance_test(benchmark_init_shutdown benchmark_init_shutdown.cpp)
if(TARGET benchmark_init_shutdown)
  target_link_libraries(benchmark_init_shutdown ${PROJECT_NAME})
endif()

add_performance_test(benchmark_node benchmark_node.cpp)
if(TARGET benchmark_node)
  target_link_libraries(benchmark_node ${PROJECT_NAME})
endif()
