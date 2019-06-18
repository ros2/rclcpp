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
# :type target: boolean
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

  add_library(${ARGS_TARGET}_component SHARED ${ARG_SRCS})
  set_target_properties(${ARGS_TARGET}_component PROPERTIES EXCLUDE_FROM_ALL ${ARGS_EXCLUDE_FROM_ALL})
  set(libs
     ${ARGS_TARGET}_component)
  add_executable(${ARGS_TARGET} ../../rclcpp/rclcpp_components/src/node_main.cpp)
  #set_target_properties(${ARGS_TARGET} PROPERTIES OUTPUT_NAME ${ARGS_TARGET})
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(libs
      "-Wl,--no-as-needed"
      ${libs}
      "-Wl,--as-needed")
  endif()
  target_link_libraries(${ARGS_TARGET}
    ${libs})
  ament_target_dependencies(${ARGS_TARGET}
    "rclcpp"
    "class_loader"
    "rclcpp_components")
  install(TARGETS
	  ${ARGS_TARGET}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

