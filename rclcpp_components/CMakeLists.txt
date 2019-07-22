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

add_library(
  component_manager
  STATIC
  src/component_manager.cpp
)
ament_target_dependencies(component_manager
  "ament_index_cpp"
  "class_loader"
  "composition_interfaces"
  "rclcpp"
  "rcpputils"
)

add_executable(
  component_container
  src/component_container.cpp
)
target_link_libraries(component_container component_manager)
ament_target_dependencies(component_container
  "rclcpp"
)

set(node_main_template_install_dir "share/${PROJECT_NAME}")
install(FILES
  src/node_main.cpp.in
  DESTINATION ${node_main_template_install_dir})

add_executable(
  component_container_mt
  src/component_container_mt.cpp
)
target_link_libraries(component_container_mt component_manager)
ament_target_dependencies(component_container_mt
  "rclcpp"
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(component_container "stdc++fs")
  target_link_libraries(component_container_mt "stdc++fs")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(components "")
  add_library(test_component SHARED test/components/test_component.cpp)
  ament_target_dependencies(test_component
    "class_loader"
    "rclcpp"
    "rclcpp_components")
  #rclcpp_components_register_nodes(test_component "test_rclcpp_components::TestComponent")
  set(components "${components}test_rclcpp_components::TestComponentFoo;$<TARGET_FILE:test_component>\n")
  set(components "${components}test_rclcpp_components::TestComponentBar;$<TARGET_FILE:test_component>\n")
  set(components "${components}test_rclcpp_components::TestComponentNoNode;$<TARGET_FILE:test_component>\n")

  file(GENERATE
    OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/rclcpp_components/${PROJECT_NAME}"
    CONTENT "${components}")

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  ament_add_gtest(test_component_manager test/test_component_manager.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_component_manager)
    target_link_libraries(test_component_manager component_manager)
    target_include_directories(test_component_manager PRIVATE src)
  endif()

  ament_add_gtest(test_component_manager_api test/test_component_manager_api.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_component_manager_api)
    target_link_libraries(test_component_manager_api component_manager)
    target_include_directories(test_component_manager_api PRIVATE src)
  endif()
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
ament_package(CONFIG_EXTRAS rclcpp_components-extras.cmake.in)
