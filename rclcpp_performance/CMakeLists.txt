cmake_minimum_required(VERSION 3.5)

project(rclcpp_performance)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ament_python_install_package(${PROJECT_NAME})

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/logs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MatchingPublisher.msg"
  ADD_LINTER_TESTS
)

find_package(rclcpp REQUIRED)

add_executable(communication_performance src/communication_performance.cpp)
ament_target_dependencies(communication_performance
  "rclcpp"
  "rosidl_generator_cpp")
rosidl_target_interfaces(communication_performance
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(rclcpp)

ament_package()

install(
  PROGRAMS scripts/posprocess_logging
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS communication_performance
  DESTINATION lib/${PROJECT_NAME}
)
