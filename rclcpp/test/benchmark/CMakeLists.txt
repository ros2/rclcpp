# Give cppcheck hints about macro definitions coming from outside this package
get_target_property(
  ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS
  performance_test_fixture::performance_test_fixture
  INTERFACE_INCLUDE_DIRECTORIES)

function(add_rclcpp_benchmark NAME SOURCE)
  cmake_parse_arguments(RCLCPP_BENCHMARK "" "LIBRARIES" "")
  if(RCLCPP_UNPARSED_ARGUMENTS)
    message(
      FATAL_ERROR
      "Unrecognized arguments for 'add_rclcpp_benchmark'"
      " (${RCLCPP_BENCHMARK_UNPARSED_ARGUMENTS})")
    return()
  endif()
  find_package(${rmw_implementation} REQUIRED)
  message(STATUS "Adding ${NAME} for '${rmw_implementation}'")
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

  set(full_benchmark_name ${NAME}${target_suffix})
  add_performance_test(${full_benchmark_name} ${SOURCE})
  if(TARGET ${full_benchmark_name})
    target_link_libraries(${full_benchmark_name} ${PROJECT_NAME} ${LIBRARIES})
  endif()
endfunction()

# Add new benchmarks inside this macro
macro(rclcpp_benchmarks)
  add_rclcpp_benchmark(benchmark_init_shutdown benchmark_init_shutdown.cpp)
  add_rclcpp_benchmark(benchmark_node benchmark_node.cpp)
endmacro()

call_for_each_rmw_implementation(rclcpp_benchmarks)
