find_package(performance_test_fixture REQUIRED)

add_performance_test(benchmark_init_shutdown benchmark_init_shutdown.cpp)
if(TARGET benchmark_init_shutdown)
  target_link_libraries(benchmark_init_shutdown ${PROJECT_NAME})
endif()

add_performance_test(benchmark_node benchmark_node.cpp)
if(TARGET benchmark_node)
  target_link_libraries(benchmark_node ${PROJECT_NAME})
endif()
