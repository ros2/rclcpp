# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
