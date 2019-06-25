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
# usage: rclcpp_components_add_node(
#              <target> [EXCLUDE_FROM_ALL] <source1> [source2 ...])

# :param EXCLUDE_FROM_ALL: exclude the target from default build target
# :type EXCLUDE_FROM_ALL: boolean
# :param target: the name of the shared library target
# :type target: string
# :param sourceN: the list of source files for executable and shared library
# :type sourceN: list of strings
#
function(rclcpp_components_add_node)
  cmake_parse_arguments(
    ARGS
    "EXCLUDE_FROM_ALL"
    ""
    ""
    ${ARGN})
  list(GET ARGS_UNPARSED_ARGUMENTS 0 target)
  list(REMOVE_AT ARGS_UNPARSED_ARGUMENTS 0)
  set(sourceN ${ARGS_UNPARSED_ARGUMENTS})
  add_library(${target} SHARED ${sourceN})
  target_compile_definitions(${target} PRIVATE "${PROJECT_NAME}_BUILDING_DLL")
  set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL ${ARGS_EXCLUDE_FROM_ALL})
  add_executable(${target}_main ../../rclcpp/rclcpp_components/src/node_main.cpp)
  set_target_properties(${target}_main PROPERTIES OUTPUT_NAME ${target})
  set(lib ${target})
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(lib
      "-Wl,--no-as-needed"
      ${target}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(${target}_main
    ${lib})
  ament_target_dependencies(${target}_main
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
    ${target}_main
    DESTINATION lib/${PROJECT_NAME})
endfunction()
