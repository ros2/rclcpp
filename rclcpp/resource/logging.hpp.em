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

#include <sstream>
#include <type_traits>

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"

// These are used for compiling out logging macros lower than a minimum severity.
#define RCLCPP_LOG_MIN_SEVERITY_DEBUG 0
#define RCLCPP_LOG_MIN_SEVERITY_INFO 1
#define RCLCPP_LOG_MIN_SEVERITY_WARN 2
#define RCLCPP_LOG_MIN_SEVERITY_ERROR 3
#define RCLCPP_LOG_MIN_SEVERITY_FATAL 4
#define RCLCPP_LOG_MIN_SEVERITY_NONE 5

#define RCLCPP_FIRST_ARG(N, ...) N
#define RCLCPP_ALL_BUT_FIRST_ARGS(N, ...) __VA_ARGS__

/**
 * \def RCLCPP_LOG_MIN_SEVERITY
 * Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * in your build options to compile out anything below that severity.
 * Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
 */
#ifndef RCLCPP_LOG_MIN_SEVERITY
#define RCLCPP_LOG_MIN_SEVERITY RCLCPP_LOG_MIN_SEVERITY_DEBUG
#endif

@{
from collections import OrderedDict
from copy import deepcopy
from rcutils.logging import feature_combinations
from rcutils.logging import get_suffix_from_features
from rcutils.logging import severities
from rcutils.logging import throttle_args
from rcutils.logging import throttle_params

throttle_args['condition_before'] = 'RCUTILS_LOG_CONDITION_THROTTLE_BEFORE(clock, duration)'
del throttle_params['get_time_point_value']
throttle_params['clock'] = 'rclcpp::Clock that will be used to get the time point.'
throttle_params.move_to_end('clock', last=False)

rclcpp_feature_combinations = OrderedDict()
for combinations, feature in feature_combinations.items():
    # skip feature combinations using 'named'
    if 'named' in combinations:
        continue
    rclcpp_feature_combinations[combinations] = feature
# add a stream variant for each available feature combination
stream_arg = 'stream_arg'
for combinations, feature in list(rclcpp_feature_combinations.items()):
    combinations = ('stream', ) + combinations
    feature = deepcopy(feature)
    feature.params[stream_arg] = 'The argument << into a stringstream'
    rclcpp_feature_combinations[combinations] = feature

def get_rclcpp_suffix_from_features(features):
    suffix = get_suffix_from_features(features)
    if 'stream' in features:
        suffix = '_STREAM' + suffix
    return suffix
}@
@[for severity in severities]@
/** @@name Logging macros for severity @(severity).
 */
///@@{
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_@(severity))
// empty logging macros for severity @(severity) when being disabled at compile time
@[ for feature_combination in rclcpp_feature_combinations.keys()]@
@{suffix = get_rclcpp_suffix_from_features(feature_combination)}@
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_@(severity)@(suffix)(...)
@[ end for]@

#else
@[ for feature_combination in rclcpp_feature_combinations.keys()]@
@{suffix = get_rclcpp_suffix_from_features(feature_combination)}@
// The RCLCPP_@(severity)@(suffix) macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_@(severity)@(suffix)
 * Log a message with severity @(severity)@
@[ if rclcpp_feature_combinations[feature_combination].doc_lines]@
 with the following conditions:
@[ else]@
.
@[ end if]@
@[ for doc_line in rclcpp_feature_combinations[feature_combination].doc_lines]@
 * @(doc_line)
@[ end for]@
 * \param logger The `rclcpp::Logger` to use
@[ for param_name, doc_line in rclcpp_feature_combinations[feature_combination].params.items()]@
 * \param @(param_name) @(doc_line)
@[ end for]@
@[ if 'stream' not in feature_combination]@
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
@[ end if]@
 */
@{params = rclcpp_feature_combinations[feature_combination].params.keys()}@
#define RCLCPP_@(severity)@(suffix)(logger@(''.join([', ' + p for p in params]))@
@[ if 'stream' not in feature_combination]@
, ...@
@[ end if]@
) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv<typename std::remove_reference<decltype(logger)>::type>::type, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
@[ if 'throttle' in feature_combination]@ \
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_@(severity)@(suffix) could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
@[ end if] \
@[ if 'stream' in feature_combination]@
    std::stringstream ss; \
    ss << @(stream_arg); \
@[ end if]@
    RCUTILS_LOG_@(severity)@(get_suffix_from_features(feature_combination))_NAMED( \
@{params = ['get_time_point' if p == 'clock' and 'throttle' in feature_combination else p for p in params]}@
@[ if params]@
@(''.join(['      ' + p + ', \\\n' for p in params if p != stream_arg]))@
@[ end if]@
      logger.get_name(), \
@[ if 'stream' not in feature_combination]@
      __VA_ARGS__); \
@[ else]@
      "%s", ss.str().c_str()); \
@[ end if]@
  } while (0)

@[ end for]@
#endif
///@@}

@[end for]@

#endif  // RCLCPP__LOGGING_HPP_
