# copied from rclcpp/rclcpp-extras.cmake

set(rclcpp_node_main_SRC "${rclcpp_DIR}/../../../src/rclcpp/node_main.cpp")

function(rclcpp_create_node_main node_library_target)
  if(NOT TARGET ${node_library_target})
    message(FATAL_ERROR "rclcpp_create_node_main() the first argument must be a valid target name")
  endif()
  set(executable_name_ ${node_library_target}_node)
  add_executable(${executable_name_} ${rclcpp_node_main_SRC})
  target_link_libraries(${executable_name_} ${node_library_target})
  install(TARGETS ${executable_name_} DESTINATION bin)
endfunction()
