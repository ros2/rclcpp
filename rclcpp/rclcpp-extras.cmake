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

# copied from rclcpp/rclcpp-extras.cmake

# register ament_package() hook for node plugins once
macro(_rclcpp_register_package_hook)
  if(NOT DEFINED _RCLCPP_PACKAGE_HOOK_REGISTERED)
    set(_RCLCPP_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    ament_register_extension("ament_package" "rclcpp"
      "rclcpp_package_hook.cmake")
  endif()
endmacro()

include("${rclcpp_DIR}/rclcpp_create_node_main.cmake")
include("${rclcpp_DIR}/rclcpp_register_node_plugins.cmake")
