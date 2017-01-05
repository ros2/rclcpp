cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

macro(targets)
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  get_rcl_lifecycle_information("${rmw_implementation}" "rcl_lifecycle${target_suffix}")

  ### CPP High level library
  add_library(rclcpp_lifecycle${target_suffix}
    SHARED
    src/lifecycle_node.cpp
    src/node_interfaces/lifecycle_node_interface.cpp
    src/state.cpp
    src/transition.cpp
  )
  ament_target_dependencies(rclcpp_lifecycle${target_suffix}
    "lifecycle_msgs"
    "rcl_lifecycle${target_suffix}"
    "rclcpp${target_suffix}")

  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(rclcpp_lifecycle${target_suffix} PRIVATE "RCLCPP_LIFECYCLE_BUILDING_DLL")

  install(TARGETS
    rclcpp_lifecycle${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lifecycle_node test/test_lifecycle_node.cpp)
  if(TARGET test_lifecycle_node)
    target_include_directories(test_lifecycle_node PUBLIC
      ${rcl_lifecycle_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${rclcpp_lifecycle_INCLUDE_DIRS}
    )
    target_link_libraries(test_lifecycle_node ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_state_machine_info test/test_state_machine_info.cpp)
  if(TARGET test_state_machine_info)
    target_include_directories(test_state_machine_info PUBLIC
      ${rcl_lifecycle_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${rclcpp_lifecycle_INCLUDE_DIRS}
    )
    target_link_libraries(test_state_machine_info ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_register_custom_callbacks test/test_register_custom_callbacks.cpp)
  if(TARGET test_register_custom_callbacks)
    target_include_directories(test_register_custom_callbacks PUBLIC
      ${rcl_lifecycle_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${rclcpp_lifecycle_INCLUDE_DIRS}
    )
    target_link_libraries(test_register_custom_callbacks ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_callback_exceptions test/test_callback_exceptions.cpp)
  if(TARGET test_callback_exceptions)
    target_include_directories(test_callback_exceptions PUBLIC
      ${rcl_lifecycle_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${rclcpp_lifecycle_INCLUDE_DIRS}
    )
    target_link_libraries(test_callback_exceptions ${PROJECT_NAME})
  endif()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(rcl_lifecycle)
ament_export_dependencies(lifecycle_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

install(DIRECTORY include/
  DESTINATION include)
