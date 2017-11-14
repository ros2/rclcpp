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

// These are used for compiling out logging macros lower than a minimum severity.
#define RCLCPP_LOG_MIN_SEVERITY_DEBUG 0
#define RCLCPP_LOG_MIN_SEVERITY_INFO 1
#define RCLCPP_LOG_MIN_SEVERITY_WARN 2
#define RCLCPP_LOG_MIN_SEVERITY_ERROR 3
#define RCLCPP_LOG_MIN_SEVERITY_FATAL 4
#define RCLCPP_LOG_MIN_SEVERITY_NONE 5

/**
 * \def RCLCPP_LOG_MIN_SEVERITY
 * Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * in your build options to compile out anything below that severity.
 * Use RCUTILS_LOG_MIN_SEVERITY_NONE to compile out all macros.
 */
#ifndef RCLCPP_LOG_MIN_SEVERITY
#define RCLCPP_LOG_MIN_SEVERITY RCLCPP_LOG_MIN_SEVERITY_DEBUG
#endif


@{
from rcutils.logging import feature_combinations
from rcutils.logging import get_macro_parameters
from rcutils.logging import get_suffix_from_features
from rcutils.logging import severities

excluded_features = ['named', 'throttle']
def is_supported_feature_combination(feature_combination):
    is_excluded = any([ef in feature_combination for ef in excluded_features])
    return not is_excluded
}@
@[for severity in severities]@
/** @@name Logging macros for severity @(severity).
 */
///@@{
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_@(severity))
// empty logging macros for severity @(severity) when being disabled at compile time
@[ for feature_combination in [fc for fc in feature_combinations if is_supported_feature_combination(fc)]]@
@{suffix = get_suffix_from_features(feature_combination)}@
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_@(severity)@(suffix)(...)
@[ end for]@

#else
@[ for feature_combination in [fc for fc in feature_combinations if is_supported_feature_combination(fc)]]@
@{suffix = get_suffix_from_features(feature_combination)}@
/**
 * \def RCLCPP_@(severity)@(suffix)
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
#define RCLCPP_@(severity)@(suffix)(name, @(''.join([p + ', ' for p in get_macro_parameters(feature_combination).keys()]))...) \
  RCUTILS_LOG_@(severity)@(suffix)_NAMED( \
@{params = get_macro_parameters(feature_combination).keys()}@
@[ if params]@
@(''.join(['    ' + p + ', \\\n' for p in params]))@
@[ end if]@
    std::string(name).c_str(), \
    __VA_ARGS__)

@[ end for]@
#endif
///@@}

@[end for]@

#endif  // RCLCPP__LOGGING_HPP_
