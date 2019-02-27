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
# Configures ros client library with custom settings.
# The custom settings are all related to library symbol visibility, see:
# https://gcc.gnu.org/wiki/Visibility
# http://www.ibm.com/developerworks/aix/library/au-aix-symbol-visibility/
#
# Below code is heavily referenced from a similar functionality in rmw:
# https://github.com/ros2/rmw/blob/master/rmw/cmake/configure_rmw_library.cmake
#
# :param library_target: the library target
# :type library_target: string
# :param LANGUAGE: Optional flag for the language of the library.
#   Allowed values are "C" and "CXX". The default is "CXX".
# :type LANGUAGE: string
#
# @public
#
macro(configure_rclcpp library_target)
  cmake_parse_arguments(_ARG "" "LANGUAGE" "" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "configure_rclcpp() called with unused arguments: ${_ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT _ARG_LANGUAGE)
    set(_ARG_LANGUAGE "CXX")
  endif()

  if(_ARG_LANGUAGE STREQUAL "C")
    # Set the visibility to hidden by default if possible
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
      # Set the visibility of symbols to hidden by default for gcc and clang
      # (this is already the default on Windows)
      set_target_properties(${library_target}
        PROPERTIES
        COMPILE_FLAGS "-fvisibility=hidden"
      )
    endif()

  elseif(_ARG_LANGUAGE STREQUAL "CXX")
    # Set the visibility to hidden by default if possible
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      # Set the visibility of symbols to hidden by default for gcc and clang
      # (this is already the default on Windows)
      set_target_properties(${library_target}
        PROPERTIES
        COMPILE_FLAGS "-fvisibility=hidden -fvisibility-inlines-hidden"
      )
    endif()

  else()
    message(FATAL_ERROR "configure_rclcpp() called with unsupported LANGUAGE: '${_ARG_LANGUAGE}'")
  endif()
endmacro()
