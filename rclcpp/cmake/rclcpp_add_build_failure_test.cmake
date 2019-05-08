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
# Register a test which tries to compile a file and expects it to fail to build.
#
# This will create two targets, one by the given target name and a test target
# which has the same name prefixed with `test_`.
# For example, if target is `should_not_compile__use_const_argument` then there
# will be an executable target called `should_not_compile__use_const_argument`
# and a test target called `test_should_not_compile__use_const_argument`.
#
# :param target: the name of the target to be created
# :type target: string
# :param ARGN: the list of source files to be used to create the test executable
# :type ARGN: list of strings
#
macro(rclcpp_add_build_failure_test target)
  if(${ARGC} EQUAL 0)
    message(
      FATAL_ERROR
      "rclcpp_add_build_failure_test() requires a target name and "
      "at least one source file")
  endif()

  add_executable(${target} ${ARGN})
  set_target_properties(${target}
    PROPERTIES
      EXCLUDE_FROM_ALL TRUE
      EXCLUDE_FROM_DEFAULT_BUILD TRUE)

  add_test(
    NAME test_${target}
    COMMAND
      ${CMAKE_COMMAND}
      --build .
      --target ${target}
      --config $<CONFIGURATION>
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  set_tests_properties(test_${target}
    PROPERTIES
      WILL_FAIL TRUE
      LABELS "build_failure"
  )
endmacro()
