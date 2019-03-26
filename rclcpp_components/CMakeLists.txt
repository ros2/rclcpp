cmake_minimum_required(VERSION 3.5)

project(rclcpp_components)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(
  component_container
  src/component_manager.cpp
  src/component_container.cpp
)

ament_target_dependencies(component_container
  "ament_index_cpp"
  "class_loader"
  "composition_interfaces"
  "rclcpp"
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(component_container "stdc++fs")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(
  TARGETS component_container
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install cmake
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_dependencies(class_loader)
ament_export_dependencies(rclcpp)
ament_package(CONFIG_EXTRAS rclcpp_components-extras.cmake)
