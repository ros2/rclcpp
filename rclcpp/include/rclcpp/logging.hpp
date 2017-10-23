// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__LOGGING_HPP_
#define RCLCPP__LOGGING_HPP_

#include "rcutils/logging_macros.h"

#ifdef ROS_PACKAGE_NAME
#define RCLCPP_CONSOLE_PACKAGE_NAME ROS_PACKAGE_NAME
#else
#define RCLCPP_CONSOLE_PACKAGE_NAME "unknown_package"
#endif

#define RCLCPP_CONSOLE_NAME_PREFIX RCLCPP_CONSOLE_PACKAGE_NAME
#define RCLCPP_CONSOLE_DEFAULT_NAME RCLCPP_CONSOLE_NAME_PREFIX

#define ROS_INFO(...) RCUTILS_LOG_INFO_NAMED(RCLCPP_CONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define ROS_INFO_NAMED(name, ...) RCUTILS_LOG_INFO_NAMED( \
  (std::string(RCLCPP_CONSOLE_DEFAULT_NAME) + "." + name).c_str(), __VA_ARGS__)

#define ROS_INFO_FULLNAMED(name, ...) RCUTILS_LOG_INFO_NAMED( \
  std::string(name).c_str(), __VA_ARGS__)

#endif  // RCLCPP__LOGGING_HPP_
