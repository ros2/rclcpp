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

#
# Get all information about rclcpp for a specific RMW implementation.
#
# It sets the common variables _DEFINITIONS, _INCLUDE_DIRS and _LIBRARIES
# with the given prefix.
#
# :param rmw_implementation: the RMW implementation name
# :type rmw_implementation: string
# :param var_prefix: the prefix of all output variable names
# :type var_prefix: string
#
macro(get_rclcpp_information rmw_implementation var_prefix)
  # pretend to be a "package"
  # so that the variables can be used by various functions / macros
  set(${var_prefix}_FOUND TRUE)

  # Get rcl using the existing macro
  if(NOT target_suffix STREQUAL "")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()

  # include directories
  normalize_path(${var_prefix}_INCLUDE_DIRS
    "${rclcpp_DIR}/../../../include")

  # libraries
  set(_libs)
  # search for library relative to this CMake file
  set(_library_target "rclcpp")
  get_available_rmw_implementations(_rmw_impls)
  list(LENGTH _rmw_impls _rmw_impls_length)
  if(_rmw_impls_length GREATER 1)
    set(_library_target "${_library_target}__${rmw_implementation}")
  endif()
  set(_lib "NOTFOUND")
  find_library(
    _lib NAMES "${_library_target}"
    PATHS "${rclcpp_DIR}/../../../lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
  )
  if(NOT _lib)
    # warn about not existing library and ignore it
    message(WARNING "Package 'rclcpp' doesn't contain the library '${_library_target}'")
  elseif(NOT IS_ABSOLUTE "${_lib}")
    # the found library must be an absolute path
    message(FATAL_ERROR "Package 'rclcpp' found the library '${_library_target}' at '${_lib}' which is not an absolute path")
  elseif(NOT EXISTS "${_lib}")
    # the found library must exist
    message(FATAL_ERROR "Package 'rclcpp' found the library '${_lib}' which doesn't exist")
  else()
    list(APPEND _libs "${_lib}")
  endif()

  # dependencies
  set(_exported_dependencies
    "rcl_interfaces"
    "rcl${target_suffix}"
    "rosidl_generator_cpp"
    "rosidl_typesupport_cpp")
  set(${var_prefix}_DEFINITIONS)
  foreach(_dep ${_exported_dependencies})
    if(NOT ${_dep}_FOUND)
      find_package("${_dep}" QUIET REQUIRED)
    endif()
    if(${_dep}_DEFINITIONS)
      list_append_unique(${var_prefix}_DEFINITIONS "${${_dep}_DEFINITIONS}")
    endif()
    if(${_dep}_INCLUDE_DIRS)
      list_append_unique(${var_prefix}_INCLUDE_DIRS "${${_dep}_INCLUDE_DIRS}")
    endif()
    if(${_dep}_LIBRARIES)
      list(APPEND _libs "${${_dep}_LIBRARIES}")
    endif()
  endforeach()
  if(_libs)
    ament_libraries_deduplicate(_libs "${_libs}")
  endif()
  set(${var_prefix}_LIBRARIES "${_libs}")
endmacro()
