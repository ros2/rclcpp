find_package(performance_test_fixture REQUIRED)

# These benchmarks are only being created and run for the default RMW
# implementation. We are looking to test the performance of the ROS 2 code, not
# the underlying middleware.

add_performance_test(
  benchmark_action_client
  benchmark_action_client.cpp
  TIMEOUT 120)
if(TARGET benchmark_action_client)
  target_link_libraries(benchmark_action_client ${PROJECT_NAME})
  ament_target_dependencies(benchmark_action_client rclcpp test_msgs)
endif()

add_performance_test(
  benchmark_action_server
  benchmark_action_server.cpp
  TIMEOUT 120)
if(TARGET benchmark_action_server)
  target_link_libraries(benchmark_action_server ${PROJECT_NAME})
  ament_target_dependencies(benchmark_action_server rclcpp test_msgs)
endif()
