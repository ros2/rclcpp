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
macro(rclcpp_components_add_node target)
  if(${ARGC} GREATER 0)
    _rclcpp_components_register_package_hook()
    set(_sources_list)
    set(SHARED_LIB_NAME "${target}_component")
    foreach(_arg ${ARGN})
      if(_arg IN_LIST _sources_list)
        message(
          FATAL_ERROR
          "rclcpp_components_add_node() the sources names "
          "must be unique (multiple '${_arg}')")
      endif()
      list(APPEND _sources_list "${_arg}")
    endforeach()
    add_library(${SHARED_LIB_NAME} SHARED ${_sources_list})
    add_executable(${target} ${_sources_list})
  endif()
endmacro()
