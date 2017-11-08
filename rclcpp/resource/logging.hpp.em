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

/**
 * \def RCLCPP_LOG_MIN_SEVERITY
 * Define RCLCPP_LOG_MIN_SEVERITY=RCUTILS_LOG_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * in your build options to compile out anything below that severity.
 */
#ifndef RCLCPP_LOG_MIN_SEVERITY
#define RCLCPP_LOG_MIN_SEVERITY RCUTILS_LOG_SEVERITY_DEBUG
#endif


@{
from rcutils.logging import feature_combinations
from rcutils.logging import get_macro_parameters
from rcutils.logging import get_suffix_from_features
from rcutils.logging import severities
}@
@[for severity in severities]@
/** @@name Logging macros for severity @(severity).
 */
///@@{
#if (RCLCPP_LOG_MIN_SEVERITY > RCUTILS_LOG_SEVERITY_@(severity))
// empty logging macros for severity @(severity) when being disabled at compile time
@[ for feature_combination in [fc for fc in feature_combinations if 'named' not in fc]]@
@{suffix = get_suffix_from_features(feature_combination)}@
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define ROS_@(severity)@(suffix)(...)
#define ROS_@(severity)@(suffix)_NAMED(name, ...)
#define ROS_@(severity)@(suffix)_FULLNAMED(name, ...)
@[ end for]@

#else
@[ for feature_combination in [fc for fc in feature_combinations if 'named' not in fc]]@
@{suffix = get_suffix_from_features(feature_combination)}@
/**
 * \def ROS_@(severity)@(suffix)
 * Log a message with severity @(severity)@
@[ if feature_combinations[feature_combination].doc_lines]@
 with the following conditions:
@[ else]@
.
@[ end if]@
@[ for doc_line in feature_combinations[feature_combination].doc_lines]@
 * @(doc_line)
@[ end for]@
@[ for param_name, doc_line in feature_combinations[feature_combination].params.items()]@
 * \param @(param_name) @(doc_line)
@[ end for]@
 * \param ... The format string, followed by the variable arguments for the format string
 */
#define ROS_@(severity)@(suffix)(...) RCUTILS_LOG_@(severity)@(suffix)_NAMED(RCLCPP_CONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define ROS_@(severity)@(suffix)_NAMED(name, @(''.join([p + ', ' for p in get_macro_parameters(feature_combination).keys()]))...) \
  RCUTILS_LOG_@(severity)@(suffix)_NAMED( \
@{params = get_macro_parameters(feature_combination).keys()}@
@[ if params]@
@(''.join(['    ' + p + ', \\\n' for p in params]))@
@[ end if]@
    (std::string(RCLCPP_CONSOLE_DEFAULT_NAME) + "." + name).c_str(), \
    __VA_ARGS__)

#define ROS_@(severity)@(suffix)_FULLNAMED(name, ...) RCUTILS_LOG_@(severity)@(suffix)_NAMED( \
  std::string(name).c_str(), __VA_ARGS__)

@[ end for]@
#endif
///@@}

@[end for]@

#endif  // RCLCPP__LOGGING_HPP_
