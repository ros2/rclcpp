find_package(performance_test_fixture REQUIRED)

#
# Add a rmw-specific performance benchmark test from performance_test_fixture
#
# :param NAME: the target name which will also be used as the test name
# :type NAME: string
# :param SOURCE: the benchmark test target's source file
# :type SOURCE: string
# :param AMENT_DEPENDS: the ament dependincies for the benchmark test target
# :type list of strings
# :param LIBRARIES: the additional libraries the target needs to be linked
#    against
# :type list of strings
# :param TEST_OPTIONS: arguments to pass directly to add_performance_test
# :type list of strings
function(add_rclcpp_benchmark NAME SOURCE)
  set(multiValueArgs AMENT_DEPENDS LIBRARIES TEST_OPTIONS)
  cmake_parse_arguments(
    RCLCPP_BENCHMARK
    ""
    ""
    "${multiValueArgs}"
    "${ARGN}")
  if(RCLCPP_BENCHMARK_UNPARSED_ARGUMENTS)
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
  add_performance_test(
    ${full_benchmark_name}
    ${SOURCE}
    ${RCLCPP_BENCHMARK_TEST_OPTIONS}
    ENV ${rmw_implementation_env_var})
  if(TARGET ${full_benchmark_name})
    if(RCLCPP_BENCHMARK_AMENT_DEPENDS)
      ament_target_dependencies(
        ${full_benchmark_name}
        ${RCLCPP_BENCHMARK_AMENT_DEPENDS})
    endif()
    target_link_libraries(
      ${full_benchmark_name}
      ${PROJECT_NAME}
      ${RCLCPP_BENCHMARK_LIBRARIES})
  endif()
endfunction()

# Add new benchmarks inside this macro
macro(rclcpp_benchmarks)
  add_rclcpp_benchmark(benchmark_init_shutdown benchmark_init_shutdown.cpp)

  set(SKIP_TEST "")
  if(${rmw_implementation} MATCHES "(.*)connext(.*)")
    set(SKIP_TEST "SKIP_TEST")
  endif()
  add_rclcpp_benchmark(
    benchmark_node
    benchmark_node.cpp
    TEST_OPTIONS ${SKIP_TEST})
endmacro()

call_for_each_rmw_implementation(rclcpp_benchmarks)
