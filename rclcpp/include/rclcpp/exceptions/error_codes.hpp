// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXCEPTIONS__ERROR_CODES_HPP_
#define RCLCPP__EXCEPTIONS__ERROR_CODES_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace exceptions
{

/// Error severity levels
enum class ErrorSeverity : uint8_t
{
  UNKNOWN = 0,   ///< Unknown or unclassified error
  INFO = 1,      ///< Informational, may not require action
  WARNING = 2,   ///< Potential issue, operation may succeed
  ERROR = 3,     ///< Operation failed, recoverable
  FATAL = 4      ///< Operation failed, unrecoverable
};

/// Error category for grouping related errors
enum class ErrorCategory : uint16_t
{
  UNKNOWN = 0,     ///< Unknown category
  NODE = 1,        ///< Node-related errors (1000-1999)
  TOPIC = 2,       ///< Topic and pub/sub errors (2000-2999)
  SERVICE = 3,     ///< Service-related errors (3000-3999)
  PARAMETER = 4,   ///< Parameter handling errors (4000-4999)
  QOS = 5,         ///< Quality of Service errors (5000-5999)
  EXECUTOR = 6,    ///< Executor and callback errors (6000-6999)
  TIMER = 7,       ///< Timer-related errors (7000-7999)
  ACTION = 8,      ///< Action-related errors (8000-8999)
  INTERNAL = 9     ///< Internal/system errors (9000-9999)
};

/// Specific error codes for rclcpp exceptions
enum class ErrorCode : uint32_t
{
  // Unknown/Generic (0)
  UNKNOWN = 0,

  // Node Errors (1000-1999)
  NODE_NAME_INVALID = 1001,
  NODE_NAMESPACE_INVALID = 1002,
  NODE_INIT_FAILED = 1003,
  NODE_ALREADY_EXISTS = 1004,
  NODE_NOT_FOUND = 1005,
  NODE_NAME_EMPTY = 1006,
  NODE_NAME_RESERVED = 1007,

  // Topic Errors (2000-2999)
  TOPIC_NAME_INVALID = 2001,
  TOPIC_NOT_AVAILABLE = 2002,
  PUBLISHER_CREATION_FAILED = 2003,
  SUBSCRIBER_CREATION_FAILED = 2004,
  TOPIC_ALREADY_EXISTS = 2005,
  TOPIC_TYPE_MISMATCH = 2006,
  TOPIC_NAME_EMPTY = 2007,

  // Service Errors (3000-3999)
  SERVICE_NAME_INVALID = 3001,
  SERVICE_NOT_AVAILABLE = 3002,
  SERVICE_CREATION_FAILED = 3003,
  CLIENT_CREATION_FAILED = 3004,
  SERVICE_CALL_FAILED = 3005,
  SERVICE_TIMEOUT = 3006,
  SERVICE_TYPE_MISMATCH = 3007,

  // Parameter Errors (4000-4999)
  PARAMETER_NOT_DECLARED = 4001,
  PARAMETER_TYPE_MISMATCH = 4002,
  PARAMETER_READ_ONLY = 4003,
  PARAMETER_INVALID_VALUE = 4004,
  PARAMETER_ALREADY_DECLARED = 4005,
  PARAMETER_NAME_INVALID = 4006,
  PARAMETER_RANGE_VIOLATION = 4007,

  // QoS Errors (5000-5999)
  QOS_INCOMPATIBLE = 5001,
  QOS_RELIABILITY_MISMATCH = 5002,
  QOS_DURABILITY_MISMATCH = 5003,
  QOS_DEADLINE_MISSED = 5004,
  QOS_LIVELINESS_LOST = 5005,
  QOS_INVALID_PROFILE = 5006,

  // Executor Errors (6000-6999)
  EXECUTOR_NODE_ALREADY_ADDED = 6001,
  EXECUTOR_CALLBACK_ERROR = 6002,
  EXECUTOR_SPIN_ERROR = 6003,
  EXECUTOR_INVALID_STATE = 6004,
  CALLBACK_GROUP_ERROR = 6005,

  // Timer Errors (7000-7999)
  TIMER_CREATION_FAILED = 7001,
  TIMER_CANCELLED = 7002,
  TIMER_INVALID_PERIOD = 7003,

  // Action Errors (8000-8999)
  ACTION_SERVER_ERROR = 8001,
  ACTION_CLIENT_ERROR = 8002,
  ACTION_GOAL_REJECTED = 8003,
  ACTION_CANCELLED = 8004,

  // Internal Errors (9000-9999)
  RCL_ERROR = 9001,
  RMW_ERROR = 9002,
  ALLOCATION_FAILED = 9003,
  INTERNAL_ERROR = 9004
};

/// Get the error category from an error code
/**
 * \param[in] code The error code to categorize
 * \return The category of the error
 */
RCLCPP_PUBLIC
constexpr ErrorCategory
get_error_category(ErrorCode code) noexcept
{
  uint32_t code_value = static_cast<uint32_t>(code);
  if (code_value == 0) {
    return ErrorCategory::UNKNOWN;
  }
  uint32_t category_value = code_value / 1000;
  if (category_value > 9) {
    return ErrorCategory::UNKNOWN;
  }
  return static_cast<ErrorCategory>(category_value);
}

/// Get the severity level for an error code
/**
 * \param[in] code The error code
 * \return The severity level (defaults to ERROR for most codes)
 */
RCLCPP_PUBLIC
constexpr ErrorSeverity
get_error_severity(ErrorCode code) noexcept
{
  // Most errors are recoverable ERROR level
  // Fatal errors are explicitly marked
  switch (code) {
    case ErrorCode::ALLOCATION_FAILED:
    case ErrorCode::INTERNAL_ERROR:
      return ErrorSeverity::FATAL;
    case ErrorCode::UNKNOWN:
      return ErrorSeverity::UNKNOWN;
    default:
      return ErrorSeverity::ERROR;
  }
}

/// Get a human-readable name for an error code
/**
 * \param[in] code The error code
 * \return String name of the error code
 */
RCLCPP_PUBLIC
std::string
get_error_code_name(ErrorCode code);

/// Get a human-readable description for an error code
/**
 * \param[in] code The error code
 * \return Description of what the error means
 */
RCLCPP_PUBLIC
std::string
get_error_code_description(ErrorCode code);

/// Get the category name as a string
/**
 * \param[in] category The error category
 * \return String name of the category
 */
RCLCPP_PUBLIC
std::string
get_category_name(ErrorCategory category);

/// Get the severity name as a string
/**
 * \param[in] severity The error severity
 * \return String name of the severity level
 */
RCLCPP_PUBLIC
std::string
get_severity_name(ErrorSeverity severity);

/// Check if an error code is in a specific category
/**
 * \param[in] code The error code to check
 * \param[in] category The category to check against
 * \return true if the code belongs to the category
 */
RCLCPP_PUBLIC
constexpr bool
is_error_in_category(ErrorCode code, ErrorCategory category) noexcept
{
  return get_error_category(code) == category;
}

}  // namespace exceptions
}  // namespace rclcpp

#endif  // RCLCPP__EXCEPTIONS__ERROR_CODES_HPP_
