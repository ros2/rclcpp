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

// These are used for compiling out logging macros lower than a minimum severity.
#define RCLCPP_LOG_MIN_SEVERITY_DEBUG 0
#define RCLCPP_LOG_MIN_SEVERITY_INFO 1
#define RCLCPP_LOG_MIN_SEVERITY_WARN 2
#define RCLCPP_LOG_MIN_SEVERITY_ERROR 3
#define RCLCPP_LOG_MIN_SEVERITY_FATAL 4
#define RCLCPP_LOG_MIN_SEVERITY_NONE 5

#define RCLCPP_STATIC_ASSERT_LOGGER(logger) \
  do { \
    static_assert( \
      ::std::is_convertible_v<decltype(logger), ::rclcpp::Logger>, \
      "First argument to logging macros must be an rclcpp::Logger"); \
  } while (0)

/**
 * \def RCLCPP_LOG
 * Log a message with given severity.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG(severity, logger, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_NAMED(severity, (logger).get_name(), __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_ONCE
 * Log a message with given severity with the following condition:
 * - All log calls except the first one are ignored.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_ONCE(severity, logger, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_ONCE_NAMED(severity, (logger).get_name(), __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_EXPRESSION
 * Log a message with given severity with the following condition:
 * - Log calls are ignored when the expression evaluates to false.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_EXPRESSION(severity, logger, expression, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_EXPRESSION_NAMED(severity, expression, (logger).get_name(), __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_FUNCTION
 * Log a message with given severity with the following condition:
 * - Log calls are ignored when the function returns false.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_FUNCTION(severity, logger, function, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_FUNCTION_NAMED(severity, function, (logger).get_name(), __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_SKIPFIRST
 * Log a message with given severity with the following condition:
 * - The first log call is ignored but all subsequent calls are processed.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_SKIPFIRST(severity, logger, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_SKIPFIRST_NAMED(severity, (logger).get_name(), __VA_ARGS__); \
  } while (0)

#define RCLCPP_LOG_TIME_POINT_FUNC(clock) \
  [&c = clock](rcutils_time_point_value_t * time_point)->rcutils_ret_t { \
    try { \
      *time_point = c.now().nanoseconds(); \
    } catch (...) { \
      RCUTILS_SAFE_FWRITE_TO_STDERR( \
      "[rclcpp|logging.hpp] RCLCPP_DEBUG_THROTTLE could not get current time stamp\n"); \
      return RCUTILS_RET_ERROR; \
    } \
    return RCUTILS_RET_OK; \
  }

/**
 * \def RCLCPP_LOG_THROTTLE
 * Log a message with given severity with the following condition:
 * - Log calls are ignored if the last logged message is not longer ago than the specified duration.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_THROTTLE(severity, logger, clock, duration, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_THROTTLE_NAMED( \
      severity, \
      RCLCPP_LOG_TIME_POINT_FUNC(clock), \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_SKIPFIRST_THROTTLE
 * Log a message with given severity with the following conditions:
 * - The first log call is ignored but all subsequent calls are processed.
 * - Log calls are ignored if the last logged message is not longer ago than the specified duration.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_LOG_SKIPFIRST_THROTTLE(severity, logger, clock, duration, ...) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    RCUTILS_LOG_SKIPFIRST_THROTTLE_NAMED( \
      severity, \
      RCLCPP_LOG_TIME_POINT_FUNC(clock), \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM
 * Log a message with given severity.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM(severity, logger, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_NAMED(severity, (logger).get_name(), "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_ONCE
 * Log a message with given severity with the following condition:
 * - All log calls except the first one are ignored.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_ONCE(severity, logger, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ONCE_NAMED(severity, (logger).get_name(), "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_EXPRESSION
 * Log a message with given severity with the following condition:
 * - Log calls are being ignored when the expression evaluates to false.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_EXPRESSION(severity, logger, expression, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_EXPRESSION_NAMED( \
      severity, \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_FUNCTION
 * Log a message with given severity with the following condition:
 * - Log calls are being ignored when the function returns false.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_FUNCTION(severity, logger, function, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FUNCTION_NAMED( \
      severity, \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_SKIPFIRST
 * Log a message with given severity with the following condition:
 * - The first log call is ignored but all subsequent calls are processed.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_SKIPFIRST(severity, logger, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_SKIPFIRST_NAMED( \
      severity, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_THROTTLE
 * Log a message with given severity with the following condition:
 * - Log calls are ignored if the last logged message is not longer ago than the specified duration.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_THROTTLE(severity, logger, clock, duration, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_THROTTLE_NAMED( \
      severity, \
      RCLCPP_LOG_TIME_POINT_FUNC(clock), \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 * Log a message with given severity with the following conditions:
 * - The first log call is ignored but all subsequent calls are processed.
 * - Log calls are ignored if the last logged message is not longer ago than the specified duration.
 *
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(severity, logger, clock, duration, stream_arg) \
  do { \
    RCLCPP_STATIC_ASSERT_LOGGER(logger); \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_SKIPFIRST_THROTTLE_NAMED( \
      severity, \
      RCLCPP_LOG_TIME_POINT_FUNC(clock), \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

/**
 * \def RCLCPP_LOG_MIN_SEVERITY
 * Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * in your build options to compile out anything below that severity.
 * Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
 */
#ifndef RCLCPP_LOG_MIN_SEVERITY
#define RCLCPP_LOG_MIN_SEVERITY RCLCPP_LOG_MIN_SEVERITY_DEBUG
#endif

/** @name Logging macros for severity DEBUG.
 */
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_DEBUG)
// empty logging macros for severity DEBUG when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(...)

#else
/**
 * \def RCLCPP_DEBUG
 * \copydoc RCLCPP_LOG
 */
#define RCLCPP_DEBUG(logger, ...) RCLCPP_LOG(RCUTILS_LOG_SEVERITY_DEBUG, logger, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_ONCE
 * \copydoc RCLCPP_LOG_ONCE
 */
#define RCLCPP_DEBUG_ONCE(logger, ...) \
  RCLCPP_LOG_ONCE(RCUTILS_LOG_SEVERITY_DEBUG, logger, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_EXPRESSION
 * \copydoc RCLCPP_LOG_EXPRESSION
 */
#define RCLCPP_DEBUG_EXPRESSION(logger, expression, ...) \
  RCLCPP_LOG_EXPRESSION(RCUTILS_LOG_SEVERITY_DEBUG, logger, expression, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_FUNCTION
 * \copydoc RCLCPP_LOG_FUNCTION
 */
#define RCLCPP_DEBUG_FUNCTION(logger, function, ...) \
  RCLCPP_LOG_FUNCTION(RCUTILS_LOG_SEVERITY_DEBUG, logger, function, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_SKIPFIRST
 * \copydoc RCLCPP_LOG_SKIPFIRST
 */
#define RCLCPP_DEBUG_SKIPFIRST(logger, ...) \
  RCLCPP_LOG_SKIPFIRST(RCUTILS_LOG_SEVERITY_DEBUG, logger, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_THROTTLE
 * \copydoc RCLCPP_LOG_THROTTLE
 */
#define RCLCPP_DEBUG_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_THROTTLE(RCUTILS_LOG_SEVERITY_DEBUG, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_SKIPFIRST_THROTTLE
 */
#define RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_DEBUG, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_DEBUG_STREAM
 * \copydoc RCLCPP_LOG_STREAM
 */
#define RCLCPP_DEBUG_STREAM(logger, stream_arg) \
  RCLCPP_LOG_STREAM(RCUTILS_LOG_SEVERITY_DEBUG, logger, stream_arg)

/**
 * \def RCLCPP_DEBUG_STREAM_ONCE
 * \copydoc RCLCPP_LOG_STREAM_ONCE
 */
#define RCLCPP_DEBUG_STREAM_ONCE(logger, stream_arg) \
  RCLCPP_LOG_STREAM_ONCE(RCUTILS_LOG_SEVERITY_DEBUG, logger, stream_arg)

/**
 * \def RCLCPP_DEBUG_STREAM_EXPRESSION
 * \copydoc RCLCPP_LOG_STREAM_EXPRESSION
 */
#define RCLCPP_DEBUG_STREAM_EXPRESSION(logger, expression, stream_arg) \
  RCLCPP_LOG_STREAM_EXPRESSION(RCUTILS_LOG_SEVERITY_DEBUG, logger, expression, stream_arg)

/**
 * \def RCLCPP_DEBUG_STREAM_FUNCTION
 * \copydoc RCLCPP_LOG_STREAM_FUNCTION
 */
#define RCLCPP_DEBUG_STREAM_FUNCTION(logger, function, stream_arg) \
  RCLCPP_LOG_STREAM_FUNCTION(RCUTILS_LOG_SEVERITY_DEBUG, logger, function, stream_args)

/**
 * \def RCLCPP_DEBUG_STREAM_SKIPFIRST
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST
 */
#define RCLCPP_DEBUG_STREAM_SKIPFIRST(logger, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST(RCUTILS_LOG_SEVERITY_DEBUG, logger, stream_arg)

/**
 * \def RCLCPP_DEBUG_STREAM_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_THROTTLE
 */
#define RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_THROTTLE(RCUTILS_LOG_SEVERITY_DEBUG, logger, clock, duration, stream_arg)

/**
 * \def RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 */
#define RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_DEBUG, logger, clock, duration, \
    stream_arg)

#endif

/** @name Logging macros for severity INFO.
 */
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_INFO)
// empty logging macros for severity INFO when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(...)

#else
/**
 * \def RCLCPP_INFO
 * \copydoc RCLCPP_LOG
 */
#define RCLCPP_INFO(logger, ...) RCLCPP_LOG(RCUTILS_LOG_SEVERITY_INFO, logger, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_ONCE
 * \copydoc RCLCPP_LOG_ONCE
 */
#define RCLCPP_INFO_ONCE(logger, ...) \
  RCLCPP_LOG_ONCE(RCUTILS_LOG_SEVERITY_INFO, logger, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_EXPRESSION
 * \copydoc RCLCPP_LOG_EXPRESSION
 */
#define RCLCPP_INFO_EXPRESSION(logger, expression, ...) \
  RCLCPP_LOG_EXPRESSION(RCUTILS_LOG_SEVERITY_INFO, logger, expression, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_FUNCTION
 * \copydoc RCLCPP_LOG_FUNCTION
 */
#define RCLCPP_INFO_FUNCTION(logger, function, ...) \
  RCLCPP_LOG_FUNCTION(RCUTILS_LOG_SEVERITY_INFO, logger, function, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_SKIPFIRST
 * \copydoc RCLCPP_LOG_SKIPFIRST
 */
#define RCLCPP_INFO_SKIPFIRST(logger, ...) \
  RCLCPP_LOG_SKIPFIRST(RCUTILS_LOG_SEVERITY_INFO, logger, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_THROTTLE
 * \copydoc RCLCPP_LOG_THROTTLE
 */
#define RCLCPP_INFO_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_THROTTLE(RCUTILS_LOG_SEVERITY_INFO, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_SKIPFIRST_THROTTLE
 */
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_INFO, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_INFO_STREAM
 * \copydoc RCLCPP_LOG_STREAM
 */
#define RCLCPP_INFO_STREAM(logger, stream_arg) \
  RCLCPP_LOG_STREAM(RCUTILS_LOG_SEVERITY_INFO, logger, stream_arg)

/**
 * \def RCLCPP_INFO_STREAM_ONCE
 * \copydoc RCLCPP_LOG_STREAM_ONCE
 */
#define RCLCPP_INFO_STREAM_ONCE(logger, stream_arg) \
  RCLCPP_LOG_STREAM_ONCE(RCUTILS_LOG_SEVERITY_INFO, logger, stream_arg)

/**
 * \def RCLCPP_INFO_STREAM_EXPRESSION
 * \copydoc RCLCPP_LOG_STREAM_EXPRESSION
 */
#define RCLCPP_INFO_STREAM_EXPRESSION(logger, expression, stream_arg) \
  RCLCPP_LOG_STREAM_EXPRESSION(RCUTILS_LOG_SEVERITY_INFO, logger, expression, stream_arg)

/**
 * \def RCLCPP_INFO_STREAM_FUNCTION
 * \copydoc RCLCPP_LOG_STREAM_FUNCTION
 */
#define RCLCPP_INFO_STREAM_FUNCTION(logger, function, stream_arg) \
  RCLCPP_LOG_STREAM_FUNCTION(RCUTILS_LOG_SEVERITY_INFO, logger, function, stream_args)

/**
 * \def RCLCPP_INFO_STREAM_SKIPFIRST
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST
 */
#define RCLCPP_INFO_STREAM_SKIPFIRST(logger, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST(RCUTILS_LOG_SEVERITY_INFO, logger, stream_arg)

/**
 * \def RCLCPP_INFO_STREAM_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_THROTTLE
 */
#define RCLCPP_INFO_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_THROTTLE(RCUTILS_LOG_SEVERITY_INFO, logger, clock, duration, stream_arg)

/**
 * \def RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 */
#define RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_INFO, logger, clock, duration, \
    stream_arg)

#endif

/** @name Logging macros for severity WARN.
 */
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_WARN)
// empty logging macros for severity WARN when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(...)

#else
/**
 * \def RCLCPP_WARN
 * \copydoc RCLCPP_LOG
 */
#define RCLCPP_WARN(logger, ...) RCLCPP_LOG(RCUTILS_LOG_SEVERITY_WARN, logger, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_ONCE
 * \copydoc RCLCPP_LOG_ONCE
 */
#define RCLCPP_WARN_ONCE(logger, ...) \
  RCLCPP_LOG_ONCE(RCUTILS_LOG_SEVERITY_WARN, logger, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_EXPRESSION
 * \copydoc RCLCPP_LOG_EXPRESSION
 */
#define RCLCPP_WARN_EXPRESSION(logger, expression, ...) \
  RCLCPP_LOG_EXPRESSION(RCUTILS_LOG_SEVERITY_WARN, logger, expression, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_FUNCTION
 * \copydoc RCLCPP_LOG_FUNCTION
 */
#define RCLCPP_WARN_FUNCTION(logger, function, ...) \
  RCLCPP_LOG_FUNCTION(RCUTILS_LOG_SEVERITY_WARN, logger, function, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_SKIPFIRST
 * \copydoc RCLCPP_LOG_SKIPFIRST
 */
#define RCLCPP_WARN_SKIPFIRST(logger, ...) \
  RCLCPP_LOG_SKIPFIRST(RCUTILS_LOG_SEVERITY_WARN, logger, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_THROTTLE
 * \copydoc RCLCPP_LOG_THROTTLE
 */
#define RCLCPP_WARN_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_THROTTLE(RCUTILS_LOG_SEVERITY_WARN, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_SKIPFIRST_THROTTLE
 */
#define RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_WARN, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_WARN_STREAM
 * \copydoc RCLCPP_LOG_STREAM
 */
#define RCLCPP_WARN_STREAM(logger, stream_arg) \
  RCLCPP_LOG_STREAM(RCUTILS_LOG_SEVERITY_WARN, logger, stream_arg)

/**
 * \def RCLCPP_WARN_STREAM_ONCE
 * \copydoc RCLCPP_LOG_STREAM_ONCE
 */
#define RCLCPP_WARN_STREAM_ONCE(logger, stream_arg) \
  RCLCPP_LOG_STREAM_ONCE(RCUTILS_LOG_SEVERITY_WARN, logger, stream_arg)

/**
 * \def RCLCPP_WARN_STREAM_EXPRESSION
 * \copydoc RCLCPP_LOG_STREAM_EXPRESSION
 */
#define RCLCPP_WARN_STREAM_EXPRESSION(logger, expression, stream_arg) \
  RCLCPP_LOG_STREAM_EXPRESSION(RCUTILS_LOG_SEVERITY_WARN, logger, expression, stream_arg)

/**
 * \def RCLCPP_WARN_STREAM_FUNCTION
 * \copydoc RCLCPP_LOG_STREAM_FUNCTION
 */
#define RCLCPP_WARN_STREAM_FUNCTION(logger, function, stream_arg) \
  RCLCPP_LOG_STREAM_FUNCTION(RCUTILS_LOG_SEVERITY_WARN, logger, function, stream_args)

/**
 * \def RCLCPP_WARN_STREAM_SKIPFIRST
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST
 */
#define RCLCPP_WARN_STREAM_SKIPFIRST(logger, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST(RCUTILS_LOG_SEVERITY_WARN, logger, stream_arg)

/**
 * \def RCLCPP_WARN_STREAM_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_THROTTLE
 */
#define RCLCPP_WARN_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_THROTTLE(RCUTILS_LOG_SEVERITY_WARN, logger, clock, duration, stream_arg)

/**
 * \def RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 */
#define RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_WARN, logger, clock, duration, \
    stream_arg)

#endif

/** @name Logging macros for severity ERROR.
 */
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_ERROR)
// empty logging macros for severity ERROR when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(...)

#else
/**
 * \def RCLCPP_ERROR
 * \copydoc RCLCPP_LOG
 */
#define RCLCPP_ERROR(logger, ...) RCLCPP_LOG(RCUTILS_LOG_SEVERITY_ERROR, logger, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_ONCE
 * \copydoc RCLCPP_LOG_ONCE
 */
#define RCLCPP_ERROR_ONCE(logger, ...) \
  RCLCPP_LOG_ONCE(RCUTILS_LOG_SEVERITY_ERROR, logger, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_EXPRESSION
 * \copydoc RCLCPP_LOG_EXPRESSION
 */
#define RCLCPP_ERROR_EXPRESSION(logger, expression, ...) \
  RCLCPP_LOG_EXPRESSION(RCUTILS_LOG_SEVERITY_ERROR, logger, expression, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_FUNCTION
 * \copydoc RCLCPP_LOG_FUNCTION
 */
#define RCLCPP_ERROR_FUNCTION(logger, function, ...) \
  RCLCPP_LOG_FUNCTION(RCUTILS_LOG_SEVERITY_ERROR, logger, function, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_SKIPFIRST
 * \copydoc RCLCPP_LOG_SKIPFIRST
 */
#define RCLCPP_ERROR_SKIPFIRST(logger, ...) \
  RCLCPP_LOG_SKIPFIRST(RCUTILS_LOG_SEVERITY_ERROR, logger, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_THROTTLE
 * \copydoc RCLCPP_LOG_THROTTLE
 */
#define RCLCPP_ERROR_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_THROTTLE(RCUTILS_LOG_SEVERITY_ERROR, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_SKIPFIRST_THROTTLE
 */
#define RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_ERROR, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_ERROR_STREAM
 * \copydoc RCLCPP_LOG_STREAM
 */
#define RCLCPP_ERROR_STREAM(logger, stream_arg) \
  RCLCPP_LOG_STREAM(RCUTILS_LOG_SEVERITY_ERROR, logger, stream_arg)

/**
 * \def RCLCPP_ERROR_STREAM_ONCE
 * \copydoc RCLCPP_LOG_STREAM_ONCE
 */
#define RCLCPP_ERROR_STREAM_ONCE(logger, stream_arg) \
  RCLCPP_LOG_STREAM_ONCE(RCUTILS_LOG_SEVERITY_ERROR, logger, stream_arg)

/**
 * \def RCLCPP_ERROR_STREAM_EXPRESSION
 * \copydoc RCLCPP_LOG_STREAM_EXPRESSION
 */
#define RCLCPP_ERROR_STREAM_EXPRESSION(logger, expression, stream_arg) \
  RCLCPP_LOG_STREAM_EXPRESSION(RCUTILS_LOG_SEVERITY_ERROR, logger, expression, stream_arg)

/**
 * \def RCLCPP_ERROR_STREAM_FUNCTION
 * \copydoc RCLCPP_LOG_STREAM_FUNCTION
 */
#define RCLCPP_ERROR_STREAM_FUNCTION(logger, function, stream_arg) \
  RCLCPP_LOG_STREAM_FUNCTION(RCUTILS_LOG_SEVERITY_ERROR, logger, function, stream_args)

/**
 * \def RCLCPP_ERROR_STREAM_SKIPFIRST
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST
 */
#define RCLCPP_ERROR_STREAM_SKIPFIRST(logger, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST(RCUTILS_LOG_SEVERITY_ERROR, logger, stream_arg)

/**
 * \def RCLCPP_ERROR_STREAM_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_THROTTLE
 */
#define RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_THROTTLE(RCUTILS_LOG_SEVERITY_ERROR, logger, clock, duration, stream_arg)

/**
 * \def RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 */
#define RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_ERROR, logger, clock, duration, \
    stream_arg)

#endif

/** @name Logging macros for severity FATAL.
 */
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_FATAL)
// empty logging macros for severity FATAL when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE(...)

#else
/**
 * \def RCLCPP_FATAL
 * \copydoc RCLCPP_LOG
 */
#define RCLCPP_FATAL(logger, ...) RCLCPP_LOG(RCUTILS_LOG_SEVERITY_FATAL, logger, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_ONCE
 * \copydoc RCLCPP_LOG_ONCE
 */
#define RCLCPP_FATAL_ONCE(logger, ...) \
  RCLCPP_LOG_ONCE(RCUTILS_LOG_SEVERITY_FATAL, logger, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_EXPRESSION
 * \copydoc RCLCPP_LOG_EXPRESSION
 */
#define RCLCPP_FATAL_EXPRESSION(logger, expression, ...) \
  RCLCPP_LOG_EXPRESSION(RCUTILS_LOG_SEVERITY_FATAL, logger, expression, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_FUNCTION
 * \copydoc RCLCPP_LOG_FUNCTION
 */
#define RCLCPP_FATAL_FUNCTION(logger, function, ...) \
  RCLCPP_LOG_FUNCTION(RCUTILS_LOG_SEVERITY_FATAL, logger, function, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_SKIPFIRST
 * \copydoc RCLCPP_LOG_SKIPFIRST
 */
#define RCLCPP_FATAL_SKIPFIRST(logger, ...) \
  RCLCPP_LOG_SKIPFIRST(RCUTILS_LOG_SEVERITY_FATAL, logger, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_THROTTLE
 * \copydoc RCLCPP_LOG_THROTTLE
 */
#define RCLCPP_FATAL_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_THROTTLE(RCUTILS_LOG_SEVERITY_FATAL, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_SKIPFIRST_THROTTLE
 */
#define RCLCPP_FATAL_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  RCLCPP_LOG_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_FATAL, logger, clock, duration, __VA_ARGS__)

/**
 * \def RCLCPP_FATAL_STREAM
 * \copydoc RCLCPP_LOG_STREAM
 */
#define RCLCPP_FATAL_STREAM(logger, stream_arg) \
  RCLCPP_LOG_STREAM(RCUTILS_LOG_SEVERITY_FATAL, logger, stream_arg)

/**
 * \def RCLCPP_FATAL_STREAM_ONCE
 * \copydoc RCLCPP_LOG_STREAM_ONCE
 */
#define RCLCPP_FATAL_STREAM_ONCE(logger, stream_arg) \
  RCLCPP_LOG_STREAM_ONCE(RCUTILS_LOG_SEVERITY_FATAL, logger, stream_arg)

/**
 * \def RCLCPP_FATAL_STREAM_EXPRESSION
 * \copydoc RCLCPP_LOG_STREAM_EXPRESSION
 */
#define RCLCPP_FATAL_STREAM_EXPRESSION(logger, expression, stream_arg) \
  RCLCPP_LOG_STREAM_EXPRESSION(RCUTILS_LOG_SEVERITY_FATAL, logger, expression, stream_arg)

/**
 * \def RCLCPP_FATAL_STREAM_FUNCTION
 * \copydoc RCLCPP_LOG_STREAM_FUNCTION
 */
#define RCLCPP_FATAL_STREAM_FUNCTION(logger, function, stream_arg) \
  RCLCPP_LOG_STREAM_FUNCTION(RCUTILS_LOG_SEVERITY_FATAL, logger, function, stream_args)

/**
 * \def RCLCPP_FATAL_STREAM_SKIPFIRST
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST
 */
#define RCLCPP_FATAL_STREAM_SKIPFIRST(logger, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST(RCUTILS_LOG_SEVERITY_FATAL, logger, stream_arg)

/**
 * \def RCLCPP_FATAL_STREAM_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_THROTTLE
 */
#define RCLCPP_FATAL_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_THROTTLE(RCUTILS_LOG_SEVERITY_FATAL, logger, clock, duration, stream_arg)

/**
 * \def RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE
 * \copydoc RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE
 */
#define RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  RCLCPP_LOG_STREAM_SKIPFIRST_THROTTLE(RCUTILS_LOG_SEVERITY_FATAL, logger, clock, duration, \
    stream_arg)

#endif

#endif  // RCLCPP__LOGGING_HPP_
