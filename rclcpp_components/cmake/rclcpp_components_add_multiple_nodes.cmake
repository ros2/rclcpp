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
function(rclcpp_components_add_multiple_nodes)
  cmake_parse_arguments(
    ARGS
    "EXCLUDE_FROM_ALL"
    "LIB_NAME"
    "NODE_NAMES;SOURCES"
    ${ARGN})
  
  set(nodes ${ARGS_NODE_NAMES})
  set(sourceN ${ARGS_SOURCES})  
  set(libraryN ${ARGS_LIB_NAME})

  add_library(${libraryN} SHARED ${sourceN})
  string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
  target_compile_definitions(${libraryN} PRIVATE "${PROJECT_NAME_UPPER}_BUILDING_DLL")
  set_target_properties(${libraryN} PROPERTIES EXCLUDE_FROM_ALL ${ARGS_EXCLUDE_FROM_ALL})

  # loop thru excecutables here and change lib name beforehand



  # ALL executable stuff below

endfunction()
