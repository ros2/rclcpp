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

# Register an rclcpp component with the ament
# resource index and create an executable.
#
# usage: rclcpp_components_register_node(
#        <target> PLUGIN <component> EXECUTABLE <node>)
#
# :param target: the shared library target
# :type target: string
# :param PLUGIN: the plugin name
# :type PLUGIN: string
# :param EXECUTABLE: the node's executable name
# :type EXECUTABLE: string
# :param RESOURCE_INDEX: the ament resource index to register the components
# :type RESOURCE_INDEX: string
#
macro(rclcpp_components_register_node target)
  cmake_parse_arguments(ARGS "" "PLUGIN;EXECUTABLE;RESOURCE_INDEX" "" ${ARGN})
  if(ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rclcpp_components_register_node() called with unused "
      "arguments: ${ARGS_UNPARSED_ARGUMENTS}")
  endif()
  if("${ARGS_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_register_node macro requires a PLUGIN argument for target ${target}")
  endif()
  if("${ARGS_EXECUTABLE}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_register_node macro requires a EXECUTABLE argument for target ${target}")
  endif()
  # default to rclcpp_components if not specified otherwise
  set(resource_index "rclcpp_components")
  if(NOT "${ARGS_RESOURCE_INDEX}" STREQUAL "")
    set(resource_index ${ARGS_RESOURCE_INDEX})
    message(STATUS "Setting component resource index to non-default value ${resource_index}")
  endif()

  set(component ${ARGS_PLUGIN})
  set(node ${ARGS_EXECUTABLE})
  _rclcpp_components_register_package_hook()
  set(_path "lib")
  set(library_name "$<TARGET_FILE_NAME:${target}>")
  if(WIN32)
    set(_path "bin")
  endif()
  set(_RCLCPP_COMPONENTS_${resource_index}__NODES
    "${_RCLCPP_COMPONENTS_${resource_index}__NODES}${component};${_path}/$<TARGET_FILE_NAME:${target}>\n")
  list(APPEND _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES ${resource_index})

  configure_file(${rclcpp_components_NODE_TEMPLATE}
    ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_configured_${node}.cpp.in)
  file(GENERATE OUTPUT ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp
    INPUT ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_configured_${node}.cpp.in)
  add_executable(${node} ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp)
  ament_target_dependencies(${node}
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
    ${node}
    DESTINATION lib/${PROJECT_NAME})
endmacro()
