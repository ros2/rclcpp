find_package(ament_cmake_gtest REQUIRED)

find_package(rmw_implementation_cmake REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

rosidl_generate_interfaces(${PROJECT_NAME}_test_msgs
  ../msg/Header.msg
  ../msg/MessageWithHeader.msg
  ../msg/String.msg
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
  SKIP_INSTALL
)
# Need the target name to depend on generated interface libraries
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_test_msgs" "rosidl_typesupport_cpp")

set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
if(WIN32)
  set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
endif()

ament_add_gtest(test_subscription_publisher_with_same_type_adapter test_subscription_publisher_with_same_type_adapter.cpp
  APPEND_LIBRARY_DIRS "${append_library_dirs}"
)
if(TARGET test_subscription_publisher_with_same_type_adapter)
  ament_target_dependencies(test_subscription_publisher_with_same_type_adapter
    "rcutils"
    "rcl_interfaces"
    "rmw"
    "rosidl_runtime_cpp"
    "rosidl_typesupport_cpp"
    "test_msgs"
  )
  target_link_libraries(test_subscription_publisher_with_same_type_adapter
    ${PROJECT_NAME}
    mimick
    ${cpp_typesupport_target})
endif()
