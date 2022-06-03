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

# Unlike other generators, this custom command depends on the target
# ${rosidl_generate_interfaces_TARGET} and not the IDL files.
# The IDL files could be generated files,as they are for .action files.
# CMake does not allow `add_custom_command()` to depend on files generated in
# a different CMake subdirectory, and this command is invoked after an
# add_subdirectory() call.
add_custom_command(
  OUTPUT ${_generated_extension_files} ${_generated_py_files} ${_generated_c_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_py_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies} ${rosidl_generate_interfaces_TARGET}
  COMMENT "Generating Python code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    DEPENDS
    ${_generated_extension_files}
    ${_generated_py_files}
    ${_generated_c_files}
  )
endif()
