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

# copied from rclcpp_components/rclcpp_components-extras.cmake

# register ament_package() hook for node plugins once.
macro(_rclcpp_components_register_package_hook)
  if(NOT DEFINED _RCLCPP_COMPONENTS_PACKAGE_HOOK_REGISTERED)
    set(_RCLCPP_COMPONENTS_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    ament_register_extension("ament_package" "rclcpp_components"
      "rclcpp_components_package_hook.cmake")
  endif()
endmacro()

get_filename_component(rclcpp_components_SHARE_DIR "${rclcpp_components_DIR}" DIRECTORY)
set(rclcpp_components_NODE_TEMPLATE "${rclcpp_components_SHARE_DIR}/node_main.cpp.in")

include("${rclcpp_components_DIR}/rclcpp_components_register_nodes.cmake")
include("${rclcpp_components_DIR}/rclcpp_components_register_node.cmake")
