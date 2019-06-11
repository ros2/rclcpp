# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Create a target which builds a shared library and also creates a target
# which builds an executable
# :param target: the executable name
# :type target: string
# :param ARGN: the list of source files for executable and shared library
# :type ARGN: list of strings
#
function(rclcpp_components_add_node target)
  set(_sources_list)
  foreach(_arg ${ARGN})
    list(APPEND _sources_list "${_arg}")
  endforeach()
  add_library(${target} SHARED ${_sources_list})
  set(libs
    ${target})
  add_executable(${target}_main ../../rclcpp/rclcpp_components/src/node_main.cpp)
  set_target_properties(${target}_main PROPERTIES OUPUT_NAME "${target}")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(libs
      "-Wl,--no-as-needed"
      ${libs}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(${target}_main
    ${libs})
  ament_target_dependencies(${target}_main
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
    ${target}_main
    DESTINATION lib/${PROJECT_NAME})
endfunction()
