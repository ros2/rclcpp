cmake_minimum_required(VERSION 3.5)

project(rclcpp_components)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual
    -Wformat=2 -Wconversion -Wshadow -Wsign-conversion -Wold-style-cast -Wcast-qual
  )
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

# Add an interface library that can be dependend upon by libraries who register components
add_library(component INTERFACE)
target_include_directories(component INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(component INTERFACE
  class_loader::class_loader
  rclcpp::rclcpp)

add_library(
  component_manager
  SHARED
  src/component_manager.cpp
)
target_include_directories(component_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(component_manager
  "ament_index_cpp"
  "class_loader"
  "composition_interfaces"
  "rclcpp"
  "rcpputils"
)
target_compile_definitions(component_manager
  PRIVATE "RCLCPP_COMPONENTS_BUILDING_LIBRARY")

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

add_executable(
  component_container_isolated
  src/component_container_isolated.cpp
)
target_link_libraries(component_container_isolated component_manager)
ament_target_dependencies(component_container_isolated
  "rclcpp"
)


if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(component_container "stdc++fs")
  target_link_libraries(component_container_mt "stdc++fs")
  target_link_libraries(component_container_isolated "stdc++fs")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_google_benchmark REQUIRED)
  find_package(benchmark REQUIRED)
  # Give cppcheck hints about macro definitions coming from outside this package
  get_target_property(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS benchmark::benchmark INTERFACE_INCLUDE_DIRECTORIES)
  ament_lint_auto_find_test_dependencies()

  set(components "")
  add_library(test_component SHARED test/components/test_component.cpp)
  target_link_libraries(test_component PRIVATE component)
  #rclcpp_components_register_nodes(test_component "test_rclcpp_components::TestComponent")
  set(components "${components}test_rclcpp_components::TestComponentFoo;$<TARGET_FILE:test_component>\n")
  set(components "${components}test_rclcpp_components::TestComponentBar;$<TARGET_FILE:test_component>\n")
  set(components "${components}test_rclcpp_components::TestComponentNoNode;$<TARGET_FILE:test_component>\n")

  # A properly formed resource only has one ';', this is used to catch an invalid resource entry
  set(invalid_components "test_rclcpp_components::TestComponentFoo;;$<TARGET_FILE:test_component>\n")

  file(GENERATE
    OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/rclcpp_components/${PROJECT_NAME}"
    CONTENT "${components}")

  file(GENERATE
    OUTPUT
    "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/rclcpp_components/invalid_${PROJECT_NAME}"
    CONTENT "${invalid_components}")

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  ament_add_gtest(test_component_manager test/test_component_manager.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_component_manager)
    target_link_libraries(test_component_manager component_manager)
  endif()

  ament_add_gtest(test_component_manager_api test/test_component_manager_api.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET test_component_manager_api)
    target_link_libraries(test_component_manager_api component_manager)
  endif()

  ament_add_google_benchmark(benchmark_components
    test/benchmark/benchmark_components.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  if(TARGET benchmark_components)
    target_link_libraries(benchmark_components component_manager)
  endif()
endif()

install(
  TARGETS component component_manager EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS component_container component_container_mt component_container_isolated
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install cmake
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(component_manager)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

# specific order: dependents before dependencies
ament_export_dependencies(ament_index_cpp)
ament_export_dependencies(class_loader)
ament_export_dependencies(composition_interfaces)
ament_export_dependencies(rclcpp)
ament_package(CONFIG_EXTRAS rclcpp_components-extras.cmake.in)
