# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and
# limitations under the License.

# Generated from generate_version_header.cmake.in
# This file is used by ament_generate_version_header()

set(GENERATED_HEADER_FILE "/home/deepanshu/deepanshu/osrf/ros2_rolling/src/ros2/rclcpp/rclcpp/cmake-build-debug/ament_generate_version_header/rclcpp/rclcpp/version.h")
set(VERSION_TEMPLATE_FILE "/home/deepanshu/deepanshu/osrf/ros2_rolling/install/ament_cmake_gen_version_h/share/ament_cmake_gen_version_h/cmake/version.h.in")

set(VERSION_MAJOR "16")
set(VERSION_MINOR "2")
set(VERSION_PATCH "0")
set(VERSION_STR "16.2.0")

set(PROJECT_NAME_UPPER "RCLCPP")

configure_file("${VERSION_TEMPLATE_FILE}" "${GENERATED_HEADER_FILE}")
