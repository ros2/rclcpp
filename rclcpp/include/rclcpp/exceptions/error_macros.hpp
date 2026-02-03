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

#ifndef RCLCPP__EXCEPTIONS__ERROR_MACROS_HPP_
#define RCLCPP__EXCEPTIONS__ERROR_MACROS_HPP_

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/exceptions/enhanced_exceptions.hpp"
#include "rclcpp/exceptions/error_codes.hpp"

/**
 * @file error_macros.hpp
 * @brief Convenience macros for throwing enhanced rclcpp exceptions
 *
 * This file provides macros that simplify throwing exceptions with
 * proper source location information. Using these macros ensures
 * consistent error handling throughout the codebase.
 */

// ============================================================================
// Basic Exception Throwing Macros
// ============================================================================

/**
 * @brief Throw an RclcppException with an error code and message
 *
 * @param code The rclcpp::exceptions::ErrorCode
 * @param msg The error message (can be a string literal or expression)
 *
 * Example:
 * @code
 * RCLCPP_THROW(ErrorCode::NODE_NAME_INVALID, "Node name cannot be empty");
 * @endcode
 */
#define RCLCPP_THROW(code, msg) \
  throw rclcpp::exceptions::RclcppException( \
    code, \
    msg)

/**
 * @brief Throw an RclcppException with error code, message, and suggestion
 *
 * @param code The rclcpp::exceptions::ErrorCode
 * @param msg The error message
 * @param suggestion A suggestion for how to fix the error
 *
 * Example:
 * @code
 * RCLCPP_THROW_WITH_SUGGESTION(
 *   ErrorCode::PARAMETER_NOT_DECLARED,
 *   "Parameter 'max_speed' not found",
 *   "Declare the parameter first with declare_parameter()");
 * @endcode
 */
#define RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion) \
  throw rclcpp::exceptions::RclcppException( \
    code, \
    msg, \
    suggestion)

/**
 * @brief Throw an RclcppException with full error suggestion structure
 *
 * @param code The rclcpp::exceptions::ErrorCode
 * @param msg The error message
 * @param suggestion An ErrorSuggestion object with alternatives
 *
 * Example:
 * @code
 * RCLCPP_THROW_WITH_FULL_SUGGESTION(
 *   ErrorCode::QOS_INCOMPATIBLE,
 *   "QoS mismatch detected",
 *   ErrorSuggestion("Check QoS profiles", {"Use RELIABLE", "Use BEST_EFFORT"}));
 * @endcode
 */
#define RCLCPP_THROW_WITH_FULL_SUGGESTION(code, msg, suggestion) \
  throw rclcpp::exceptions::RclcppException( \
    code, \
    msg, \
    rclcpp::exceptions::ErrorSuggestion(suggestion))

/**
 * @brief Throw an RclcppException with context information
 *
 * @param code The rclcpp::exceptions::ErrorCode
 * @param msg The error message
 * @param suggestion A suggestion string
 * @param context An ErrorContext object
 *
 * Example:
 * @code
 * auto ctx = ErrorContext().with_node_name("my_node").with_topic_name("/chatter");
 * RCLCPP_THROW_WITH_CONTEXT(
 *   ErrorCode::TOPIC_NAME_INVALID,
 *   "Invalid topic name",
 *   "Topic names must start with a letter",
 *   ctx);
 * @endcode
 */
#define RCLCPP_THROW_WITH_CONTEXT(code, msg, suggestion, context) \
  throw rclcpp::exceptions::RclcppException( \
    code, \
    msg, \
    rclcpp::exceptions::ErrorSuggestion(suggestion), \
    context)

// ============================================================================
// Conditional Throwing Macros
// ============================================================================

/**
 * @brief Throw if condition is true
 *
 * @param condition Boolean expression to evaluate
 * @param code Error code to use if throwing
 * @param msg Error message
 *
 * Example:
 * @code
 * RCLCPP_THROW_IF(node_name.empty(), ErrorCode::NODE_NAME_EMPTY, "Node name is empty");
 * @endcode
 */
#define RCLCPP_THROW_IF(condition, code, msg) \
  do { \
    if (condition) { \
      RCLCPP_THROW(code, msg); \
    } \
  } while (0)

/**
 * @brief Throw if condition is false
 *
 * @param condition Boolean expression to evaluate
 * @param code Error code to use if throwing
 * @param msg Error message
 *
 * Example:
 * @code
 * RCLCPP_THROW_UNLESS(is_valid_name(name), ErrorCode::NODE_NAME_INVALID, "Invalid name");
 * @endcode
 */
#define RCLCPP_THROW_UNLESS(condition, code, msg) \
  RCLCPP_THROW_IF(!(condition), code, msg)

/**
 * @brief Throw if pointer is null
 *
 * @param ptr Pointer to check
 * @param code Error code to use if null
 * @param msg Error message
 *
 * Example:
 * @code
 * RCLCPP_THROW_IF_NULL(node_ptr, ErrorCode::INTERNAL_ERROR, "Node pointer is null");
 * @endcode
 */
#define RCLCPP_THROW_IF_NULL(ptr, code, msg) \
  RCLCPP_THROW_IF((ptr) == nullptr, code, msg)

// ============================================================================
// Category-Specific Throwing Macros
// ============================================================================

/**
 * @brief Throw a node-related exception
 */
#define RCLCPP_THROW_NODE_ERROR(code, msg, suggestion) \
  static_assert( \
    static_cast<uint32_t>(code) >= 1000 && static_cast<uint32_t>(code) < 2000, \
    "Error code must be in NODE category (1000-1999)"); \
  RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion)

/**
 * @brief Throw a topic-related exception
 */
#define RCLCPP_THROW_TOPIC_ERROR(code, msg, suggestion) \
  static_assert( \
    static_cast<uint32_t>(code) >= 2000 && static_cast<uint32_t>(code) < 3000, \
    "Error code must be in TOPIC category (2000-2999)"); \
  RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion)

/**
 * @brief Throw a service-related exception
 */
#define RCLCPP_THROW_SERVICE_ERROR(code, msg, suggestion) \
  static_assert( \
    static_cast<uint32_t>(code) >= 3000 && static_cast<uint32_t>(code) < 4000, \
    "Error code must be in SERVICE category (3000-3999)"); \
  RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion)

/**
 * @brief Throw a parameter-related exception
 */
#define RCLCPP_THROW_PARAMETER_ERROR(code, msg, suggestion) \
  static_assert( \
    static_cast<uint32_t>(code) >= 4000 && static_cast<uint32_t>(code) < 5000, \
    "Error code must be in PARAMETER category (4000-4999)"); \
  RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion)

/**
 * @brief Throw a QoS-related exception
 */
#define RCLCPP_THROW_QOS_ERROR(code, msg, suggestion) \
  static_assert( \
    static_cast<uint32_t>(code) >= 5000 && static_cast<uint32_t>(code) < 6000, \
    "Error code must be in QOS category (5000-5999)"); \
  RCLCPP_THROW_WITH_SUGGESTION(code, msg, suggestion)

// ============================================================================
// Specialized Exception Throwing Macros
// ============================================================================

/**
 * @brief Throw InvalidNodeNameError
 *
 * @param name The invalid node name
 * @param reason Optional reason string
 */
#define RCLCPP_THROW_INVALID_NODE_NAME(name, ...) \
  throw rclcpp::exceptions::InvalidNodeNameError(name, ##__VA_ARGS__)

/**
 * @brief Throw InvalidNamespaceError
 *
 * @param ns The invalid namespace
 * @param reason Optional reason string
 */
#define RCLCPP_THROW_INVALID_NAMESPACE(ns, ...) \
  throw rclcpp::exceptions::InvalidNamespaceError(ns, ##__VA_ARGS__)

/**
 * @brief Throw InvalidTopicNameError
 *
 * @param name The invalid topic name
 * @param reason Optional reason string
 */
#define RCLCPP_THROW_INVALID_TOPIC_NAME(name, ...) \
  throw rclcpp::exceptions::InvalidTopicNameError(name, ##__VA_ARGS__)

/**
 * @brief Throw InvalidServiceNameError
 *
 * @param name The invalid service name
 * @param reason Optional reason string
 */
#define RCLCPP_THROW_INVALID_SERVICE_NAME(name, ...) \
  throw rclcpp::exceptions::InvalidServiceNameError(name, ##__VA_ARGS__)

/**
 * @brief Throw ServiceNotAvailableError
 *
 * @param name The service name that is not available
 */
#define RCLCPP_THROW_SERVICE_NOT_AVAILABLE(name) \
  throw rclcpp::exceptions::ServiceNotAvailableError(name)

/**
 * @brief Throw ParameterNotDeclaredException
 *
 * @param name The parameter name that was not declared
 */
#define RCLCPP_THROW_PARAMETER_NOT_DECLARED(name) \
  throw rclcpp::exceptions::ParameterNotDeclaredException(name)

/**
 * @brief Throw ParameterTypeMismatchException
 *
 * @param name The parameter name
 * @param expected The expected type name
 * @param actual The actual type name
 */
#define RCLCPP_THROW_PARAMETER_TYPE_MISMATCH(name, expected, actual) \
  throw rclcpp::exceptions::ParameterTypeMismatchException(name, expected, actual)

/**
 * @brief Throw InvalidParameterValueException
 *
 * @param name The parameter name
 * @param reason The reason the value is invalid
 */
#define RCLCPP_THROW_INVALID_PARAMETER_VALUE(name, reason) \
  throw rclcpp::exceptions::InvalidParameterValueException(name, reason)

/**
 * @brief Throw QoSIncompatibleError
 *
 * @param topic The topic name
 * @param pub_qos Publisher QoS description
 * @param sub_qos Subscriber QoS description
 */
#define RCLCPP_THROW_QOS_INCOMPATIBLE(topic, pub_qos, sub_qos) \
  throw rclcpp::exceptions::QoSIncompatibleError(topic, pub_qos, sub_qos)

// ============================================================================
// Message Formatting Helpers
// ============================================================================

/**
 * @brief Helper macro for building formatted error messages
 *
 * @param ... Stream operations to build the message
 *
 * Example:
 * @code
 * std::string msg = RCLCPP_ERROR_MSG("Invalid value " << value << " for parameter " << name);
 * @endcode
 */
#define RCLCPP_ERROR_MSG(...) \
  (static_cast<std::ostringstream &&>(std::ostringstream() << __VA_ARGS__).str())

/**
 * @brief Throw with a formatted message
 *
 * @param code The error code
 * @param ... Stream operations to build the message
 *
 * Example:
 * @code
 * RCLCPP_THROW_FMT(ErrorCode::NODE_NAME_INVALID, "Node '" << name << "' has invalid character '" << ch << "'");
 * @endcode
 */
#define RCLCPP_THROW_FMT(code, ...) \
  RCLCPP_THROW(code, RCLCPP_ERROR_MSG(__VA_ARGS__))

/**
 * @brief Throw with formatted message and suggestion
 *
 * @param code The error code
 * @param suggestion The suggestion string
 * @param ... Stream operations to build the message
 */
#define RCLCPP_THROW_FMT_WITH_SUGGESTION(code, suggestion, ...) \
  RCLCPP_THROW_WITH_SUGGESTION(code, RCLCPP_ERROR_MSG(__VA_ARGS__), suggestion)

// ============================================================================
// Context Builder Helpers
// ============================================================================

namespace rclcpp
{
namespace exceptions
{

/**
 * @brief Create an ErrorContext with fluent interface
 *
 * @return A new ErrorContext instance for chaining
 *
 * Example:
 * @code
 * auto context = make_context()
 *   .with_node_name("my_node")
 *   .with_topic_name("/chatter");
 * @endcode
 */
inline ErrorContext
make_context()
{
  return ErrorContext();
}

/**
 * @brief Create an ErrorContext starting with node name
 *
 * @param name The node name to include
 * @return ErrorContext for chaining
 */
inline ErrorContext
context_for_node(const std::string & name)
{
  return ErrorContext().with_node_name(name);
}

/**
 * @brief Create an ErrorContext starting with topic name
 *
 * @param name The topic name to include
 * @return ErrorContext for chaining
 */
inline ErrorContext
context_for_topic(const std::string & name)
{
  return ErrorContext().with_topic_name(name);
}

/**
 * @brief Create an ErrorContext starting with service name
 *
 * @param name The service name to include
 * @return ErrorContext for chaining
 */
inline ErrorContext
context_for_service(const std::string & name)
{
  return ErrorContext().with_service_name(name);
}

/**
 * @brief Create an ErrorContext starting with parameter name
 *
 * @param name The parameter name to include
 * @return ErrorContext for chaining
 */
inline ErrorContext
context_for_parameter(const std::string & name)
{
  return ErrorContext().with_parameter_name(name);
}

/**
 * @brief Create an ErrorSuggestion with primary message
 *
 * @param primary The primary suggestion message
 * @return ErrorSuggestion for use in exceptions
 */
inline ErrorSuggestion
make_suggestion(const std::string & primary)
{
  return ErrorSuggestion(primary);
}

/**
 * @brief Create an ErrorSuggestion with alternatives
 *
 * @param primary The primary suggestion message
 * @param alternatives Vector of alternative suggestions
 * @return ErrorSuggestion for use in exceptions
 */
inline ErrorSuggestion
make_suggestion(const std::string & primary, const std::vector<std::string> & alternatives)
{
  return ErrorSuggestion(primary, alternatives);
}

/**
 * @brief Create an ErrorSuggestion with documentation link
 *
 * @param primary The primary suggestion message
 * @param alternatives Vector of alternative suggestions
 * @param doc_url URL to relevant documentation
 * @return ErrorSuggestion for use in exceptions
 */
inline ErrorSuggestion
make_suggestion(
  const std::string & primary,
  const std::vector<std::string> & alternatives,
  const std::string & doc_url)
{
  return ErrorSuggestion(primary, alternatives, doc_url);
}

// ============================================================================
// Error Handler Utilities
// ============================================================================

/**
 * @brief RAII guard for collecting exception context
 *
 * Use this class to automatically add context to any exceptions
 * thrown within a scope.
 *
 * Example:
 * @code
 * void process_node(const std::string& name) {
 *   ExceptionContextGuard guard;
 *   guard.add("node_name", name);
 *   // Any exceptions thrown here will have the node_name context
 * }
 * @endcode
 */
class RCLCPP_PUBLIC ExceptionContextGuard
{
public:
  ExceptionContextGuard() = default;
  ~ExceptionContextGuard() = default;

  ExceptionContextGuard(const ExceptionContextGuard &) = delete;
  ExceptionContextGuard & operator=(const ExceptionContextGuard &) = delete;

  /**
   * @brief Add a context key-value pair
   */
  ExceptionContextGuard & add(const std::string & key, const std::string & value)
  {
    context_.with(key, value);
    return *this;
  }

  /**
   * @brief Get the accumulated context
   */
  const ErrorContext & context() const noexcept
  {
    return context_;
  }

private:
  ErrorContext context_;
};

/**
 * @brief Convert an error code to its string representation
 *
 * @param code The error code
 * @return String in format "[CODE] NAME"
 */
inline std::string
error_code_to_string(ErrorCode code)
{
  return "[" + std::to_string(static_cast<uint32_t>(code)) + "] " + get_error_code_name(code);
}

/**
 * @brief Check if an exception is of a specific error category
 *
 * @param ex The exception to check
 * @param category The category to match
 * @return true if the exception belongs to the category
 */
inline bool
is_exception_category(const RclcppException & ex, ErrorCategory category) noexcept
{
  return ex.error_category() == category;
}

/**
 * @brief Check if an exception has a specific error code
 *
 * @param ex The exception to check
 * @param code The code to match
 * @return true if the exception has the specified code
 */
inline bool
is_exception_code(const RclcppException & ex, ErrorCode code) noexcept
{
  return ex.error_code() == code;
}

}  // namespace exceptions
}  // namespace rclcpp

// ============================================================================
// Legacy Compatibility Macros
// ============================================================================

// These macros provide backward compatibility with older error handling patterns

/**
 * @brief Throw a runtime error with rclcpp exception info
 *
 * This is provided for gradual migration from std::runtime_error
 */
#define RCLCPP_RUNTIME_ERROR(msg) \
  throw rclcpp::exceptions::RclcppException( \
    rclcpp::exceptions::ErrorCode::INTERNAL_ERROR, \
    msg)

#endif  // RCLCPP__EXCEPTIONS__ERROR_MACROS_HPP_
