# Copyright 2016 Open Source Robotics Foundation, Inc.
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
# Register a node plugin with the ament resource index.
#
# The passed library can contain multiple plugins extending the node interface.
#
# :param target: the shared library target
# :type target: string
# :param ARGN: the unique plugin names being exported using class_loader
# :type ARGN: list of strings
#
macro(rclcpp_register_node_plugins target)
  if(NOT TARGET ${target})
    message(
      FATAL_ERROR
      "rclcpp_register_node_plugins() first argument "
      "'${target}' is not a target")
  endif()
  get_target_property(_target_type ${target} TYPE)
  if(NOT _target_type STREQUAL "SHARED_LIBRARY")
    message(
      FATAL_ERROR
      "rclcpp_register_node_plugins() first argument "
      "'${target}' is not a shared library target")
  endif()

  if(${ARGC} GREATER 0)
    _rclcpp_register_package_hook()
    set(_unique_names)
    foreach(_arg ${ARGN})
      if(_arg IN_LIST _unique_names)
        message(
          FATAL_ERROR
          "rclcpp_register_node_plugins() the plugin names "
          "must be unique (multiple '${_arg}')")
      endif()
      list(APPEND _unique_names "${_arg}")

      if(WIN32)
        set(_path "bin")
      else()
        set(_path "lib")
      endif()
      set(_RCLCPP__NODE_PLUGINS
        "${_RCLCPP__NODE_PLUGINS}${_arg};${_path}/$<TARGET_FILE_NAME:${target}>\n")
    endforeach()
  endif()
endmacro()
