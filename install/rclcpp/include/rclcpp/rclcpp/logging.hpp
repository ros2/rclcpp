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

/** @name Logging macros for severity DEBUG.
 */
///@{
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
// The RCLCPP_DEBUG macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG
 * Log a message with severity DEBUG.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_DEBUG_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_ONCE(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_DEBUG_ONCE_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_EXPRESSION(logger, expression, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_DEBUG_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_FUNCTION(logger, function, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_DEBUG_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_SKIPFIRST(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_DEBUG_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_DEBUG_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_DEBUG_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_DEBUG_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_DEBUG_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_DEBUG_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM
 * Log a message with severity DEBUG.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_ONCE(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_ONCE_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_EXPRESSION(logger, expression, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_FUNCTION(logger, function, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_SKIPFIRST(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_DEBUG_STREAM_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_DEBUG_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

#endif
///@}

/** @name Logging macros for severity INFO.
 */
///@{
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
// The RCLCPP_INFO macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO
 * Log a message with severity INFO.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_INFO_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_ONCE(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_INFO_ONCE_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_EXPRESSION(logger, expression, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_INFO_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_FUNCTION(logger, function, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_INFO_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_SKIPFIRST(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_INFO_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_INFO_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_INFO_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_INFO_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_INFO_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_INFO_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM
 * Log a message with severity INFO.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_ONCE(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_ONCE_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_EXPRESSION(logger, expression, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_FUNCTION(logger, function, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_SKIPFIRST(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_INFO_STREAM_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_INFO_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

#endif
///@}

/** @name Logging macros for severity WARN.
 */
///@{
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
// The RCLCPP_WARN macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN
 * Log a message with severity WARN.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_WARN_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_ONCE
 * Log a message with severity WARN with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_ONCE(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_WARN_ONCE_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_EXPRESSION
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_EXPRESSION(logger, expression, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_WARN_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_FUNCTION
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_FUNCTION(logger, function, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_WARN_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_SKIPFIRST
 * Log a message with severity WARN with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_SKIPFIRST(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_WARN_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_THROTTLE
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_WARN_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_WARN_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_SKIPFIRST_THROTTLE
 * Log a message with severity WARN with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_WARN_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_WARN_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_WARN_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM
 * Log a message with severity WARN.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_ONCE
 * Log a message with severity WARN with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_ONCE(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_ONCE_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_EXPRESSION
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_EXPRESSION(logger, expression, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_FUNCTION
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_FUNCTION(logger, function, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_SKIPFIRST
 * Log a message with severity WARN with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_SKIPFIRST(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_THROTTLE
 * Log a message with severity WARN with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_WARN_STREAM_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity WARN with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_WARN_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

#endif
///@}

/** @name Logging macros for severity ERROR.
 */
///@{
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
// The RCLCPP_ERROR macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR
 * Log a message with severity ERROR.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_ERROR_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_ONCE
 * Log a message with severity ERROR with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_ONCE(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_ERROR_ONCE_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_EXPRESSION
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_EXPRESSION(logger, expression, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_ERROR_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_FUNCTION
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_FUNCTION(logger, function, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_ERROR_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_SKIPFIRST
 * Log a message with severity ERROR with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_SKIPFIRST(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_ERROR_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_THROTTLE
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_ERROR_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_ERROR_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_SKIPFIRST_THROTTLE
 * Log a message with severity ERROR with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_ERROR_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_ERROR_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_ERROR_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM
 * Log a message with severity ERROR.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_ONCE
 * Log a message with severity ERROR with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_ONCE(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_ONCE_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_EXPRESSION
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_EXPRESSION(logger, expression, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_FUNCTION
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_FUNCTION(logger, function, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_SKIPFIRST
 * Log a message with severity ERROR with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_SKIPFIRST(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_THROTTLE
 * Log a message with severity ERROR with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_ERROR_STREAM_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity ERROR with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_ERROR_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

#endif
///@}

/** @name Logging macros for severity FATAL.
 */
///@{
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
// The RCLCPP_FATAL macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL
 * Log a message with severity FATAL.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_FATAL_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_ONCE
 * Log a message with severity FATAL with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_ONCE(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_FATAL_ONCE_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_EXPRESSION
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_EXPRESSION(logger, expression, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_FATAL_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_FUNCTION
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_FUNCTION(logger, function, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_FATAL_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_SKIPFIRST
 * Log a message with severity FATAL with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_SKIPFIRST(logger, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    RCUTILS_LOG_FATAL_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_THROTTLE
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_FATAL_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_FATAL_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_SKIPFIRST_THROTTLE
 * Log a message with severity FATAL with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 */
#define RCLCPP_FATAL_SKIPFIRST_THROTTLE(logger, clock, duration, ...) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_FATAL_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    RCUTILS_LOG_FATAL_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      __VA_ARGS__); \
  } while (0)

// The RCLCPP_FATAL_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM
 * Log a message with severity FATAL.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_ONCE
 * Log a message with severity FATAL with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_ONCE(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_ONCE_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_EXPRESSION
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param logger The `rclcpp::Logger` to use
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_EXPRESSION(logger, expression, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_EXPRESSION_NAMED( \
      expression, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_FUNCTION
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param logger The `rclcpp::Logger` to use
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_FUNCTION(logger, function, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_FUNCTION_NAMED( \
      function, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_SKIPFIRST
 * Log a message with severity FATAL with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param logger The `rclcpp::Logger` to use
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_SKIPFIRST(logger, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_SKIPFIRST_NAMED( \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_THROTTLE
 * Log a message with severity FATAL with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_FATAL_STREAM_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

// The RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity FATAL with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param logger The `rclcpp::Logger` to use
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE(logger, clock, duration, stream_arg) \
  do { \
    static_assert( \
      ::std::is_same<typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)>>, \
      typename ::rclcpp::Logger>::value, \
      "First argument to logging macros must be an rclcpp::Logger"); \
\
    auto get_time_point = [&c=clock](rcutils_time_point_value_t * time_point) -> rcutils_ret_t { \
      try { \
        *time_point = c.now().nanoseconds(); \
      } catch (...) { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
        "[rclcpp|logging.hpp] RCLCPP_FATAL_STREAM_SKIPFIRST_THROTTLE could not get current time stamp\n"); \
        return RCUTILS_RET_ERROR; \
      } \
        return RCUTILS_RET_OK; \
    }; \
 \
    std::stringstream rclcpp_stream_ss_; \
    rclcpp_stream_ss_ << stream_arg; \
    RCUTILS_LOG_FATAL_SKIPFIRST_THROTTLE_NAMED( \
      get_time_point, \
      duration, \
      (logger).get_name(), \
      "%s", rclcpp_stream_ss_.str().c_str()); \
  } while (0)

#endif
///@}


#endif  // RCLCPP__LOGGING_HPP_
