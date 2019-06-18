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
# :param EXCLUDE_FROM_ALL: exclude the target from default build target 
# :type EXCLUDE_FROM_ALL: boolean
# :param ARGS_TARGET: the name of the shared library 
# :type ARGS_TARGET
# :param ARG_SRCS: the list of source files for executable and shared library
# :type ARG_SRCS: list of strings
#
function(rclcpp_components_add_node)
  cmake_parse_arguments(
    ARGS
    "EXCLUDE_FROM_ALL"
    ""
    ""
    ${ARGN}
    )

  list(GET ARGS_UNPARSED_ARGUMENTS 0 ARGS_TARGET)
  list(REMOVE_AT ARGS_UNPARSED_ARGUMENTS 0)
  set(ARG_SRCS ${ARGS_UNPARSED_ARGUMENTS})

  add_library(${ARGS_TARGET} SHARED ${ARG_SRCS})
  set_target_properties(${ARGS_TARGET} PROPERTIES EXCLUDE_FROM_ALL ${ARGS_EXCLUDE_FROM_ALL})
  
  add_executable(${ARGS_TARGET}_main ../../rclcpp/rclcpp_components/src/node_main.cpp)
  set_target_properties(${ARGS_TARGET}_main PROPERTIES OUTPUT_NAME ${ARGS_TARGET})
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(target
      "-Wl,--no-as-needed"
      ${ARGS_TARGET}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(${ARGS_TARGET}_main
	  ${target})
  ament_target_dependencies(${ARGS_TARGET}_main
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
	  ${ARGS_TARGET}_main
    DESTINATION lib/${PROJECT_NAME})
endfunction()

