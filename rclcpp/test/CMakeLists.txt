find_package(rosidl_default_generators REQUIRED)

find_package(test_msgs REQUIRED)

include(cmake/rclcpp_add_build_failure_test.cmake)

set(TEST_RESOURCES_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_subdirectory(benchmark)
add_subdirectory(rclcpp)

ament_add_gtest(test_rclcpp_gtest_macros utils/test_rclcpp_gtest_macros.cpp)
if(TARGET test_rclcpp_gtest_macros)
  target_link_libraries(test_rclcpp_gtest_macros ${PROJECT_NAME})
endif()
