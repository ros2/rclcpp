find_package(rosidl_default_generators REQUIRED)

find_package(test_msgs REQUIRED)

include(cmake/rclcpp_add_build_failure_test.cmake)

add_subdirectory(benchmark)
add_subdirectory(rclcpp)

ament_add_gtest(test_rclcpp_gtest_macros utils/test_rclcpp_gtest_macros.cpp)
if(TARGET test_rclcpp_gtest_macros)
  target_link_libraries(test_rclcpp_gtest_macros ${PROJECT_NAME})
endif()

# Install test resources
install(
  DIRECTORY resources
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
