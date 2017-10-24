// generated from rclcpp/resource/logging.hpp.em

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

@{
import rcutils.logging
from rcutils.logging import feature_combinations
from rcutils.logging import get_macro_arguments
from rcutils.logging import get_macro_parameters
from rcutils.logging import severities
}@
@[for severity in severities]@
/** @@name Logging macros for severity @(severity).
 */
///@@{
@[ for suffix in [s for s in feature_combinations if 'NAMED' not in s]]@
#define ROS_@(severity)@(suffix)(...) RCUTILS_LOG_@(severity)@(suffix)_NAMED(RCLCPP_CONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define ROS_@(severity)@(suffix)_NAMED(name, ...) RCUTILS_LOG_@(severity)@(suffix)_NAMED( \
  (std::string(RCLCPP_CONSOLE_DEFAULT_NAME) + "." + name).c_str(), __VA_ARGS__)

#define ROS_@(severity)@(suffix)_FULLNAMED(name, ...) RCUTILS_LOG_@(severity)@(suffix)_NAMED( \
  std::string(name).c_str(), __VA_ARGS__)
@[ end for]@
///@@}

@[end for]@

#endif  // RCLCPP__LOGGING_HPP_
