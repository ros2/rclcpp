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

# register node plugins
# The internal data is stored in a project directory scoped properties to allow
# registering the components from nested scopes in CMake, where variables
# would not propagate out.
get_property(_rclcpp_components_package_resource_indices
  DIRECTORY "${PROJECT_SOURCE_DIR}"
  PROPERTY _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES
)
list(REMOVE_DUPLICATES _rclcpp_components_package_resource_indices)
foreach(resource_index ${_rclcpp_components_package_resource_indices})
  get_property(_rclcpp_components_nodes
    DIRECTORY "${PROJECT_SOURCE_DIR}"
    PROPERTY "_RCLCPP_COMPONENTS_${resource_index}__NODES"
  )
  ament_index_register_resource(
    ${resource_index} CONTENT "${_rclcpp_components_nodes}")
endforeach()

