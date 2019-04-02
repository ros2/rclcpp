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
find_package(rcpputils REQUIRED)

include_directories(include)

add_executable(
  component_container
  src/component_container.cpp
  src/component_manager.cpp
)

ament_target_dependencies(component_container
  "ament_index_cpp"
  "class_loader"
  "composition_interfaces"
  "rclcpp"
  "rcpputils"
)

add_executable(
  component_container_mt
  src/component_container_mt.cpp
  src/component_manager.cpp
)

ament_target_dependencies(component_container_mt
  "ament_index_cpp"
  "class_loader"
  "composition_interfaces"
  "rclcpp"
  "rcpputils"
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(component_container "stdc++fs")
  target_link_libraries(component_container_mt "stdc++fs")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(
  TARGETS component_container component_container_mt
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
