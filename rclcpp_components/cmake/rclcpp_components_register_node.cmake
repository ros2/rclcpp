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
#
# usage: rclcpp_components_register_node(
#              <target> PLUGIN <component> EXECUTABLE <node>)
#
# Register an rclcpp component with the ament
# resource index and create an executable.
#
# :param target: the shared library target
# :type target: string
# :param PLUGIN: the plugin name
# :type PLUGIN: string
# :type EXECUTABLE: the node's executable name
# :type EXECUTABLE: string
#
macro(rclcpp_components_register_node target)
  cmake_parse_arguments(
    ARGS
    ""
    "PLUGIN;EXECUTABLE"
    ""
    ${ARGN})
  set(component ${ARGS_PLUGIN})
  set(node ${ARGS_EXECUTABLE})
  _rclcpp_components_register_package_hook()
  set(_path "lib")
  if(WIN32)
    set(_path "bin")
    set(library_name ${target}.dll)
  elseif(APPLE)
    set(library_name lib${target}.dylib)
  else()
    set(library_name lib${target}.so)
  endif()
  set(_RCLCPP_COMPONENTS__NODES
    "${_RCLCPP_COMPONENTS__NODES}${component};${_path}/$<TARGET_FILE_NAME:${target}>\n")
  configure_file(${rclcpp_components_NODE_TEMPLATE}
    ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp @ONLY)
  add_executable(${node} ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp)
  set(lib ${target})
  # Needed so symbols aren't dropped if not used
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(lib
      "-Wl,--no-as-needed"
      ${target}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(${node}
    ${lib})
  ament_target_dependencies(${node}
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
    ${node}
    DESTINATION lib/${PROJECT_NAME})
endmacro()
