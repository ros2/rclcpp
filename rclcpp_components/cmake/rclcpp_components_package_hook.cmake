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
list(REMOVE_DUPLICATES _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES)
foreach(resource_index ${_RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES})
  ament_index_register_resource(
    ${resource_index} CONTENT "${_RCLCPP_COMPONENTS_${resource_index}__NODES}")
endforeach()

