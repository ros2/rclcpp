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

#include "rclcpp/exceptions/enhanced_exceptions.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <sstream>
#include <utility>

namespace rclcpp
{
namespace exceptions
{

// ============================================================================
// Error Code Utility Functions Implementation
// ============================================================================

std::string
get_error_code_name(ErrorCode code)
{
  switch (code) {
    case ErrorCode::UNKNOWN:
      return "UNKNOWN";
    case ErrorCode::NODE_NAME_INVALID:
      return "NODE_NAME_INVALID";
    case ErrorCode::NODE_NAMESPACE_INVALID:
      return "NODE_NAMESPACE_INVALID";
    case ErrorCode::NODE_INIT_FAILED:
      return "NODE_INIT_FAILED";
    case ErrorCode::NODE_ALREADY_EXISTS:
      return "NODE_ALREADY_EXISTS";
    case ErrorCode::NODE_NOT_FOUND:
      return "NODE_NOT_FOUND";
    case ErrorCode::NODE_NAME_EMPTY:
      return "NODE_NAME_EMPTY";
    case ErrorCode::NODE_NAME_RESERVED:
      return "NODE_NAME_RESERVED";
    case ErrorCode::TOPIC_NAME_INVALID:
      return "TOPIC_NAME_INVALID";
    case ErrorCode::TOPIC_NOT_AVAILABLE:
      return "TOPIC_NOT_AVAILABLE";
    case ErrorCode::PUBLISHER_CREATION_FAILED:
      return "PUBLISHER_CREATION_FAILED";
    case ErrorCode::SUBSCRIBER_CREATION_FAILED:
      return "SUBSCRIBER_CREATION_FAILED";
    case ErrorCode::TOPIC_ALREADY_EXISTS:
      return "TOPIC_ALREADY_EXISTS";
    case ErrorCode::TOPIC_TYPE_MISMATCH:
      return "TOPIC_TYPE_MISMATCH";
    case ErrorCode::TOPIC_NAME_EMPTY:
      return "TOPIC_NAME_EMPTY";
    case ErrorCode::SERVICE_NAME_INVALID:
      return "SERVICE_NAME_INVALID";
    case ErrorCode::SERVICE_NOT_AVAILABLE:
      return "SERVICE_NOT_AVAILABLE";
    case ErrorCode::SERVICE_CREATION_FAILED:
      return "SERVICE_CREATION_FAILED";
    case ErrorCode::CLIENT_CREATION_FAILED:
      return "CLIENT_CREATION_FAILED";
    case ErrorCode::SERVICE_CALL_FAILED:
      return "SERVICE_CALL_FAILED";
    case ErrorCode::SERVICE_TIMEOUT:
      return "SERVICE_TIMEOUT";
    case ErrorCode::SERVICE_TYPE_MISMATCH:
      return "SERVICE_TYPE_MISMATCH";
    case ErrorCode::PARAMETER_NOT_DECLARED:
      return "PARAMETER_NOT_DECLARED";
    case ErrorCode::PARAMETER_TYPE_MISMATCH:
      return "PARAMETER_TYPE_MISMATCH";
    case ErrorCode::PARAMETER_READ_ONLY:
      return "PARAMETER_READ_ONLY";
    case ErrorCode::PARAMETER_INVALID_VALUE:
      return "PARAMETER_INVALID_VALUE";
    case ErrorCode::PARAMETER_ALREADY_DECLARED:
      return "PARAMETER_ALREADY_DECLARED";
    case ErrorCode::PARAMETER_NAME_INVALID:
      return "PARAMETER_NAME_INVALID";
    case ErrorCode::PARAMETER_RANGE_VIOLATION:
      return "PARAMETER_RANGE_VIOLATION";
    case ErrorCode::QOS_INCOMPATIBLE:
      return "QOS_INCOMPATIBLE";
    case ErrorCode::QOS_RELIABILITY_MISMATCH:
      return "QOS_RELIABILITY_MISMATCH";
    case ErrorCode::QOS_DURABILITY_MISMATCH:
      return "QOS_DURABILITY_MISMATCH";
    case ErrorCode::QOS_DEADLINE_MISSED:
      return "QOS_DEADLINE_MISSED";
    case ErrorCode::QOS_LIVELINESS_LOST:
      return "QOS_LIVELINESS_LOST";
    case ErrorCode::QOS_INVALID_PROFILE:
      return "QOS_INVALID_PROFILE";
    case ErrorCode::EXECUTOR_NODE_ALREADY_ADDED:
      return "EXECUTOR_NODE_ALREADY_ADDED";
    case ErrorCode::EXECUTOR_CALLBACK_ERROR:
      return "EXECUTOR_CALLBACK_ERROR";
    case ErrorCode::EXECUTOR_SPIN_ERROR:
      return "EXECUTOR_SPIN_ERROR";
    case ErrorCode::EXECUTOR_INVALID_STATE:
      return "EXECUTOR_INVALID_STATE";
    case ErrorCode::CALLBACK_GROUP_ERROR:
      return "CALLBACK_GROUP_ERROR";
    case ErrorCode::TIMER_CREATION_FAILED:
      return "TIMER_CREATION_FAILED";
    case ErrorCode::TIMER_CANCELLED:
      return "TIMER_CANCELLED";
    case ErrorCode::TIMER_INVALID_PERIOD:
      return "TIMER_INVALID_PERIOD";
    case ErrorCode::ACTION_SERVER_ERROR:
      return "ACTION_SERVER_ERROR";
    case ErrorCode::ACTION_CLIENT_ERROR:
      return "ACTION_CLIENT_ERROR";
    case ErrorCode::ACTION_GOAL_REJECTED:
      return "ACTION_GOAL_REJECTED";
    case ErrorCode::ACTION_CANCELLED:
      return "ACTION_CANCELLED";
    case ErrorCode::RCL_ERROR:
      return "RCL_ERROR";
    case ErrorCode::RMW_ERROR:
      return "RMW_ERROR";
    case ErrorCode::ALLOCATION_FAILED:
      return "ALLOCATION_FAILED";
    case ErrorCode::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    default:
      return "UNKNOWN_ERROR_CODE";
  }
}

std::string
get_error_code_description(ErrorCode code)
{
  switch (code) {
    case ErrorCode::UNKNOWN:
      return "Unknown or unclassified error";
    case ErrorCode::NODE_NAME_INVALID:
      return "Node name does not meet ROS 2 naming requirements";
    case ErrorCode::NODE_NAMESPACE_INVALID:
      return "Namespace is invalid";
    case ErrorCode::NODE_INIT_FAILED:
      return "Node initialization failed";
    case ErrorCode::NODE_ALREADY_EXISTS:
      return "A node with this name already exists";
    case ErrorCode::NODE_NOT_FOUND:
      return "Specified node was not found";
    case ErrorCode::NODE_NAME_EMPTY:
      return "Node name cannot be empty";
    case ErrorCode::NODE_NAME_RESERVED:
      return "Node name uses a reserved prefix";
    case ErrorCode::TOPIC_NAME_INVALID:
      return "Topic name is invalid";
    case ErrorCode::TOPIC_NOT_AVAILABLE:
      return "Topic is not available";
    case ErrorCode::PUBLISHER_CREATION_FAILED:
      return "Failed to create publisher";
    case ErrorCode::SUBSCRIBER_CREATION_FAILED:
      return "Failed to create subscriber";
    case ErrorCode::TOPIC_ALREADY_EXISTS:
      return "Topic is already registered";
    case ErrorCode::TOPIC_TYPE_MISMATCH:
      return "Message type does not match the topic";
    case ErrorCode::TOPIC_NAME_EMPTY:
      return "Topic name cannot be empty";
    case ErrorCode::SERVICE_NAME_INVALID:
      return "Service name is invalid";
    case ErrorCode::SERVICE_NOT_AVAILABLE:
      return "Service is not available";
    case ErrorCode::SERVICE_CREATION_FAILED:
      return "Failed to create service";
    case ErrorCode::CLIENT_CREATION_FAILED:
      return "Failed to create service client";
    case ErrorCode::SERVICE_CALL_FAILED:
      return "Service call failed";
    case ErrorCode::SERVICE_TIMEOUT:
      return "Service call timed out";
    case ErrorCode::SERVICE_TYPE_MISMATCH:
      return "Service type does not match";
    case ErrorCode::PARAMETER_NOT_DECLARED:
      return "Parameter was not declared before use";
    case ErrorCode::PARAMETER_TYPE_MISMATCH:
      return "Parameter type does not match expected type";
    case ErrorCode::PARAMETER_READ_ONLY:
      return "Parameter is read-only and cannot be modified";
    case ErrorCode::PARAMETER_INVALID_VALUE:
      return "Parameter value is invalid";
    case ErrorCode::PARAMETER_ALREADY_DECLARED:
      return "Parameter has already been declared";
    case ErrorCode::PARAMETER_NAME_INVALID:
      return "Parameter name is invalid";
    case ErrorCode::PARAMETER_RANGE_VIOLATION:
      return "Parameter value is outside the allowed range";
    case ErrorCode::QOS_INCOMPATIBLE:
      return "QoS settings are incompatible between publisher and subscriber";
    case ErrorCode::QOS_RELIABILITY_MISMATCH:
      return "Reliability QoS settings do not match";
    case ErrorCode::QOS_DURABILITY_MISMATCH:
      return "Durability QoS settings do not match";
    case ErrorCode::QOS_DEADLINE_MISSED:
      return "QoS deadline was missed";
    case ErrorCode::QOS_LIVELINESS_LOST:
      return "QoS liveliness assertion failed";
    case ErrorCode::QOS_INVALID_PROFILE:
      return "Invalid QoS profile specified";
    case ErrorCode::EXECUTOR_NODE_ALREADY_ADDED:
      return "Node has already been added to an executor";
    case ErrorCode::EXECUTOR_CALLBACK_ERROR:
      return "Error occurred during callback execution";
    case ErrorCode::EXECUTOR_SPIN_ERROR:
      return "Error occurred during spin operation";
    case ErrorCode::EXECUTOR_INVALID_STATE:
      return "Executor is in an invalid state";
    case ErrorCode::CALLBACK_GROUP_ERROR:
      return "Callback group configuration error";
    case ErrorCode::TIMER_CREATION_FAILED:
      return "Failed to create timer";
    case ErrorCode::TIMER_CANCELLED:
      return "Timer was cancelled";
    case ErrorCode::TIMER_INVALID_PERIOD:
      return "Invalid timer period specified";
    case ErrorCode::ACTION_SERVER_ERROR:
      return "Action server error";
    case ErrorCode::ACTION_CLIENT_ERROR:
      return "Action client error";
    case ErrorCode::ACTION_GOAL_REJECTED:
      return "Action goal was rejected";
    case ErrorCode::ACTION_CANCELLED:
      return "Action was cancelled";
    case ErrorCode::RCL_ERROR:
      return "Error from RCL layer";
    case ErrorCode::RMW_ERROR:
      return "Error from RMW layer";
    case ErrorCode::ALLOCATION_FAILED:
      return "Memory allocation failed";
    case ErrorCode::INTERNAL_ERROR:
      return "Internal error";
    default:
      return "Unknown error";
  }
}

std::string
get_category_name(ErrorCategory category)
{
  switch (category) {
    case ErrorCategory::UNKNOWN:
      return "Unknown";
    case ErrorCategory::NODE:
      return "Node";
    case ErrorCategory::TOPIC:
      return "Topic";
    case ErrorCategory::SERVICE:
      return "Service";
    case ErrorCategory::PARAMETER:
      return "Parameter";
    case ErrorCategory::QOS:
      return "QoS";
    case ErrorCategory::EXECUTOR:
      return "Executor";
    case ErrorCategory::TIMER:
      return "Timer";
    case ErrorCategory::ACTION:
      return "Action";
    case ErrorCategory::INTERNAL:
      return "Internal";
    default:
      return "Unknown";
  }
}

std::string
get_severity_name(ErrorSeverity severity)
{
  switch (severity) {
    case ErrorSeverity::UNKNOWN:
      return "UNKNOWN";
    case ErrorSeverity::INFO:
      return "INFO";
    case ErrorSeverity::WARNING:
      return "WARNING";
    case ErrorSeverity::ERROR:
      return "ERROR";
    case ErrorSeverity::FATAL:
      return "FATAL";
    default:
      return "UNKNOWN";
  }
}

// ============================================================================
// ErrorSuggestion Implementation
// ============================================================================

std::string
ErrorSuggestion::format() const
{
  if (empty()) {
    return "";
  }

  std::ostringstream oss;
  if (!primary.empty()) {
    oss << primary;
  }

  if (!alternatives.empty()) {
    oss << "\n  Alternatives:";
    for (const auto & alt : alternatives) {
      oss << "\n    - " << alt;
    }
  }

  if (!documentation_url.empty()) {
    oss << "\n  Documentation: " << documentation_url;
  }

  return oss.str();
}

// ============================================================================
// ErrorContext Implementation
// ============================================================================

ErrorContext::ErrorContext(const ErrorContext & other)
: context_(other.context_)
{
}

ErrorContext::ErrorContext(ErrorContext && other) noexcept
: context_(std::move(other.context_))
{
}

ErrorContext &
ErrorContext::operator=(const ErrorContext & other)
{
  if (this != &other) {
    context_ = other.context_;
  }
  return *this;
}

ErrorContext &
ErrorContext::operator=(ErrorContext && other) noexcept
{
  if (this != &other) {
    context_ = std::move(other.context_);
  }
  return *this;
}

ErrorContext::~ErrorContext() = default;

ErrorContext &
ErrorContext::with_node_name(const std::string & name)
{
  context_["node_name"] = name;
  return *this;
}

ErrorContext &
ErrorContext::with_topic_name(const std::string & name)
{
  context_["topic_name"] = name;
  return *this;
}

ErrorContext &
ErrorContext::with_service_name(const std::string & name)
{
  context_["service_name"] = name;
  return *this;
}

ErrorContext &
ErrorContext::with_parameter_name(const std::string & name)
{
  context_["parameter_name"] = name;
  return *this;
}

ErrorContext &
ErrorContext::with_expected_type(const std::string & type)
{
  context_["expected_type"] = type;
  return *this;
}

ErrorContext &
ErrorContext::with_actual_type(const std::string & type)
{
  context_["actual_type"] = type;
  return *this;
}

ErrorContext &
ErrorContext::with_expected_value(const std::string & value)
{
  context_["expected_value"] = value;
  return *this;
}

ErrorContext &
ErrorContext::with_actual_value(const std::string & value)
{
  context_["actual_value"] = value;
  return *this;
}

ErrorContext &
ErrorContext::with_qos_profile(const std::string & profile)
{
  context_["qos_profile"] = profile;
  return *this;
}

ErrorContext &
ErrorContext::with_namespace(const std::string & ns)
{
  context_["namespace"] = ns;
  return *this;
}

ErrorContext &
ErrorContext::with(const std::string & key, const std::string & value)
{
  context_[key] = value;
  return *this;
}

std::string
ErrorContext::format() const
{
  if (context_.empty()) {
    return "";
  }

  std::ostringstream oss;
  bool first = true;
  for (const auto & [key, value] : context_) {
    if (!first) {
      oss << ", ";
    }
    oss << key << "=" << value;
    first = false;
  }
  return oss.str();
}

const std::map<std::string, std::string> &
ErrorContext::as_map() const noexcept
{
  return context_;
}

bool
ErrorContext::empty() const noexcept
{
  return context_.empty();
}

// ============================================================================
// RclcppException Implementation
// ============================================================================

struct RclcppException::Impl
{
  ErrorCode code;
  std::string message;
  ErrorSuggestion suggestion;
  std::map<std::string, std::string> context;
  const char * file_name;
  const char * function_name;
  uint32_t line;

  mutable std::string what_message;
  mutable bool what_formatted = false;
  mutable std::mutex format_mutex;
};

#if RCLCPP_HAS_SOURCE_LOCATION
RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const std::source_location & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const std::string & suggestion,
  const std::source_location & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = ErrorSuggestion(suggestion);
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const ErrorSuggestion & suggestion,
  const std::source_location & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = suggestion;
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const ErrorSuggestion & suggestion,
  const ErrorContext & context,
  const std::source_location & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = suggestion;
  impl_->context = context.as_map();
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}
#else
RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const SourceLocation & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const std::string & suggestion,
  const SourceLocation & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = ErrorSuggestion(suggestion);
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const ErrorSuggestion & suggestion,
  const SourceLocation & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = suggestion;
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}

RclcppException::RclcppException(
  ErrorCode code,
  const std::string & message,
  const ErrorSuggestion & suggestion,
  const ErrorContext & context,
  const SourceLocation & location)
: impl_(std::make_unique<Impl>())
{
  impl_->code = code;
  impl_->message = message;
  impl_->suggestion = suggestion;
  impl_->context = context.as_map();
  impl_->file_name = location.file_name();
  impl_->function_name = location.function_name();
  impl_->line = location.line();
}
#endif

RclcppException::RclcppException(const RclcppException & other)
: std::exception(other),
  impl_(std::make_unique<Impl>())
{
  impl_->code = other.impl_->code;
  impl_->message = other.impl_->message;
  impl_->suggestion = other.impl_->suggestion;
  impl_->context = other.impl_->context;
  impl_->file_name = other.impl_->file_name;
  impl_->function_name = other.impl_->function_name;
  impl_->line = other.impl_->line;
  impl_->what_formatted = false;
}

RclcppException::RclcppException(RclcppException && other) noexcept
: std::exception(std::move(other)),
  impl_(std::move(other.impl_))
{
}

RclcppException &
RclcppException::operator=(const RclcppException & other)
{
  if (this != &other) {
    std::exception::operator=(other);
    impl_ = std::make_unique<Impl>();
    impl_->code = other.impl_->code;
    impl_->message = other.impl_->message;
    impl_->suggestion = other.impl_->suggestion;
    impl_->context = other.impl_->context;
    impl_->file_name = other.impl_->file_name;
    impl_->function_name = other.impl_->function_name;
    impl_->line = other.impl_->line;
    impl_->what_formatted = false;
  }
  return *this;
}

RclcppException &
RclcppException::operator=(RclcppException && other) noexcept
{
  if (this != &other) {
    std::exception::operator=(std::move(other));
    impl_ = std::move(other.impl_);
  }
  return *this;
}

RclcppException::~RclcppException() = default;

ErrorCode
RclcppException::error_code() const noexcept
{
  return impl_->code;
}

ErrorCategory
RclcppException::error_category() const noexcept
{
  return get_error_category(impl_->code);
}

ErrorSeverity
RclcppException::error_severity() const noexcept
{
  return get_error_severity(impl_->code);
}

const std::string &
RclcppException::message() const noexcept
{
  return impl_->message;
}

const std::string &
RclcppException::suggestion() const noexcept
{
  return impl_->suggestion.primary;
}

const ErrorSuggestion &
RclcppException::full_suggestion() const noexcept
{
  return impl_->suggestion;
}

std::string
RclcppException::context() const
{
  if (impl_->context.empty()) {
    return "";
  }

  std::ostringstream oss;
  bool first = true;
  for (const auto & [key, value] : impl_->context) {
    if (!first) {
      oss << ", ";
    }
    oss << key << "=" << value;
    first = false;
  }
  return oss.str();
}

const std::map<std::string, std::string> &
RclcppException::context_map() const noexcept
{
  return impl_->context;
}

const char *
RclcppException::file_name() const noexcept
{
  return impl_->file_name ? impl_->file_name : "";
}

const char *
RclcppException::function_name() const noexcept
{
  return impl_->function_name ? impl_->function_name : "";
}

uint32_t
RclcppException::line() const noexcept
{
  return impl_->line;
}

std::string
RclcppException::location_string() const
{
  std::ostringstream oss;
  if (impl_->file_name && impl_->file_name[0] != '\0') {
    oss << impl_->file_name;
    if (impl_->line > 0) {
      oss << ":" << impl_->line;
    }
    if (impl_->function_name && impl_->function_name[0] != '\0') {
      oss << " in " << impl_->function_name << "()";
    }
  }
  return oss.str();
}

void
RclcppException::format_what_message() const
{
  std::lock_guard<std::mutex> lock(impl_->format_mutex);
  if (impl_->what_formatted) {
    return;
  }

  std::ostringstream oss;

  // Error code and message
  oss << "[" << static_cast<uint32_t>(impl_->code) << "] " << impl_->message;

  // Context (compact inline)
  if (!impl_->context.empty()) {
    oss << " (";
    bool first = true;
    for (const auto & [key, value] : impl_->context) {
      if (!first) {
        oss << ", ";
      }
      oss << key << "=" << value;
      first = false;
    }
    oss << ")";
  }

  // Suggestion
  if (!impl_->suggestion.primary.empty()) {
    oss << " | Suggestion: " << impl_->suggestion.primary;
  }

  // Location
  if (impl_->file_name && impl_->file_name[0] != '\0' && impl_->line > 0) {
    oss << " [" << impl_->file_name << ":" << impl_->line << "]";
  }

  impl_->what_message = oss.str();
  impl_->what_formatted = true;
}

const char *
RclcppException::what() const noexcept
{
  if (!impl_->what_formatted) {
    try {
      format_what_message();
    } catch (...) {
      return impl_->message.c_str();
    }
  }
  return impl_->what_message.c_str();
}

std::string
RclcppException::detailed_report() const
{
  std::ostringstream oss;

  oss << "=== ROS 2 Exception Report ===" << "\n";
  oss << "Error Code: " << static_cast<uint32_t>(impl_->code)
      << " (" << get_error_code_name(impl_->code) << ")" << "\n";
  oss << "Category: " << get_category_name(error_category()) << "\n";
  oss << "Severity: " << get_severity_name(error_severity()) << "\n";
  oss << "\n";
  oss << "Message: " << impl_->message << "\n";

  if (!impl_->context.empty()) {
    oss << "\n";
    oss << "Context:" << "\n";
    for (const auto & [key, value] : impl_->context) {
      oss << "  " << key << ": " << value << "\n";
    }
  }

  if (!impl_->suggestion.empty()) {
    oss << "\n";
    oss << "Suggestion: " << impl_->suggestion.primary << "\n";
    if (!impl_->suggestion.alternatives.empty()) {
      oss << "Alternatives:" << "\n";
      for (const auto & alt : impl_->suggestion.alternatives) {
        oss << "  - " << alt << "\n";
      }
    }
    if (!impl_->suggestion.documentation_url.empty()) {
      oss << "Documentation: " << impl_->suggestion.documentation_url << "\n";
    }
  }

  oss << "\n";
  oss << "Location: " << location_string() << "\n";
  oss << "==============================" << "\n";

  return oss.str();
}

RclcppException &
RclcppException::with_context(const std::string & key, const std::string & value)
{
  impl_->context[key] = value;
  impl_->what_formatted = false;
  return *this;
}

// ============================================================================
// Specialized Exception Implementations
// ============================================================================

#if RCLCPP_HAS_SOURCE_LOCATION
InvalidNodeNameError::InvalidNodeNameError(
  const std::string & node_name,
  const std::string & reason,
  const std::source_location & location)
: NodeException(
    ErrorCode::NODE_NAME_INVALID,
    "Invalid node name '" + node_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Node names must start with a letter or underscore, and contain only "
      "alphanumeric characters and underscores",
      {"Avoid starting with numbers", "Avoid using reserved names starting with '_'"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("node_name", node_name);
}

InvalidNamespaceError::InvalidNamespaceError(
  const std::string & namespace_name,
  const std::string & reason,
  const std::source_location & location)
: NodeException(
    ErrorCode::NODE_NAMESPACE_INVALID,
    "Invalid namespace '" + namespace_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Namespaces must be absolute (start with '/') and follow ROS 2 naming conventions",
      {"Ensure namespace starts with '/'", "Use only alphanumeric characters and underscores"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("namespace", namespace_name);
}

NodeInitializationError::NodeInitializationError(
  const std::string & node_name,
  const std::string & reason,
  const std::source_location & location)
: NodeException(
    ErrorCode::NODE_INIT_FAILED,
    "Failed to initialize node '" + node_name + "': " + reason,
    ErrorSuggestion(
      "Check that ROS 2 is properly initialized with rclcpp::init() before creating nodes",
      {"Verify rclcpp::init() was called", "Check for RMW implementation errors"}
    ),
    location)
{
  with_context("node_name", node_name);
}

InvalidTopicNameError::InvalidTopicNameError(
  const std::string & topic_name,
  const std::string & reason,
  const std::source_location & location)
: TopicException(
    ErrorCode::TOPIC_NAME_INVALID,
    "Invalid topic name '" + topic_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Topic names should start with a letter and contain only alphanumeric characters, "
      "underscores, and forward slashes",
      {"Use absolute topic names starting with '/'", "Avoid special characters"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("topic_name", topic_name);
}

InvalidServiceNameError::InvalidServiceNameError(
  const std::string & service_name,
  const std::string & reason,
  const std::source_location & location)
: ServiceException(
    ErrorCode::SERVICE_NAME_INVALID,
    "Invalid service name '" + service_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Service names follow the same rules as topic names",
      {"Use absolute service names starting with '/'", "Avoid special characters"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("service_name", service_name);
}

ServiceNotAvailableError::ServiceNotAvailableError(
  const std::string & service_name,
  const std::source_location & location)
: ServiceException(
    ErrorCode::SERVICE_NOT_AVAILABLE,
    "Service '" + service_name + "' is not available",
    ErrorSuggestion(
      "Ensure the service server is running and properly advertised",
      {
        "Check if the service server node is running",
        "Use 'ros2 service list' to see available services",
        "Verify the service name matches exactly"
      }
    ),
    location)
{
  with_context("service_name", service_name);
}

ParameterNotDeclaredException::ParameterNotDeclaredException(
  const std::string & parameter_name,
  const std::source_location & location)
: ParameterException(
    ErrorCode::PARAMETER_NOT_DECLARED,
    "Parameter '" + parameter_name + "' has not been declared",
    ErrorSuggestion(
      "Declare the parameter before use with declare_parameter<T>(name, default_value)",
      {
        "Use declare_parameter() in the node constructor",
        "Set allow_undeclared_parameters to true in NodeOptions",
        "Use has_parameter() to check existence first"
      },
      "https://docs.ros.org/en/rolling/Tutorials/Parameters/Understanding-ROS2-Parameters.html"
    ),
    location)
{
  with_context("parameter_name", parameter_name);
}

ParameterTypeMismatchException::ParameterTypeMismatchException(
  const std::string & parameter_name,
  const std::string & expected_type,
  const std::string & actual_type,
  const std::source_location & location)
: ParameterException(
    ErrorCode::PARAMETER_TYPE_MISMATCH,
    "Parameter '" + parameter_name + "' type mismatch: expected " + expected_type +
    ", got " + actual_type,
    ErrorSuggestion(
      "Use the correct type when getting the parameter value",
      {
        "Check parameter declaration for the expected type",
        "Use get_parameter_or() for type-safe access"
      }
    ),
    location)
{
  with_context("parameter_name", parameter_name);
  with_context("expected_type", expected_type);
  with_context("actual_type", actual_type);
}

InvalidParameterValueException::InvalidParameterValueException(
  const std::string & parameter_name,
  const std::string & reason,
  const std::source_location & location)
: ParameterException(
    ErrorCode::PARAMETER_INVALID_VALUE,
    "Invalid value for parameter '" + parameter_name + "': " + reason,
    ErrorSuggestion(
      "Provide a valid value that meets the parameter's constraints",
      {
        "Check parameter description for valid values",
        "Use describe_parameter() to see constraints"
      }
    ),
    location)
{
  with_context("parameter_name", parameter_name);
}

QoSIncompatibleError::QoSIncompatibleError(
  const std::string & topic_name,
  const std::string & publisher_qos,
  const std::string & subscriber_qos,
  const std::source_location & location)
: QoSException(
    ErrorCode::QOS_INCOMPATIBLE,
    "QoS incompatibility on topic '" + topic_name + "'",
    ErrorSuggestion(
      "Ensure publisher and subscriber QoS settings are compatible",
      {
        "Use matching reliability settings (reliable/best_effort)",
        "Use compatible durability settings",
        "Consider using QoS profiles like rclcpp::SensorDataQoS() for consistent settings"
      },
      "https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html"
    ),
    location)
{
  with_context("topic_name", topic_name);
  with_context("publisher_qos", publisher_qos);
  with_context("subscriber_qos", subscriber_qos);
}

CallbackGroupError::CallbackGroupError(
  const std::string & reason,
  const std::source_location & location)
: ExecutorException(
    ErrorCode::CALLBACK_GROUP_ERROR,
    "Callback group error: " + reason,
    ErrorSuggestion(
      "Review callback group configuration and ensure proper assignment",
      {
        "Check that callbacks are added to appropriate groups",
        "Verify executor handles the callback group type (mutually exclusive vs reentrant)"
      }
    ),
    location)
{
}

#else
// Fallback implementations for pre-C++20
InvalidNodeNameError::InvalidNodeNameError(
  const std::string & node_name,
  const std::string & reason,
  const SourceLocation & location)
: NodeException(
    ErrorCode::NODE_NAME_INVALID,
    "Invalid node name '" + node_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Node names must start with a letter or underscore, and contain only "
      "alphanumeric characters and underscores",
      {"Avoid starting with numbers", "Avoid using reserved names starting with '_'"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("node_name", node_name);
}

InvalidNamespaceError::InvalidNamespaceError(
  const std::string & namespace_name,
  const std::string & reason,
  const SourceLocation & location)
: NodeException(
    ErrorCode::NODE_NAMESPACE_INVALID,
    "Invalid namespace '" + namespace_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Namespaces must be absolute (start with '/') and follow ROS 2 naming conventions",
      {"Ensure namespace starts with '/'", "Use only alphanumeric characters and underscores"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("namespace", namespace_name);
}

NodeInitializationError::NodeInitializationError(
  const std::string & node_name,
  const std::string & reason,
  const SourceLocation & location)
: NodeException(
    ErrorCode::NODE_INIT_FAILED,
    "Failed to initialize node '" + node_name + "': " + reason,
    ErrorSuggestion(
      "Check that ROS 2 is properly initialized with rclcpp::init() before creating nodes",
      {"Verify rclcpp::init() was called", "Check for RMW implementation errors"}
    ),
    location)
{
  with_context("node_name", node_name);
}

InvalidTopicNameError::InvalidTopicNameError(
  const std::string & topic_name,
  const std::string & reason,
  const SourceLocation & location)
: TopicException(
    ErrorCode::TOPIC_NAME_INVALID,
    "Invalid topic name '" + topic_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Topic names should start with a letter and contain only alphanumeric characters, "
      "underscores, and forward slashes",
      {"Use absolute topic names starting with '/'", "Avoid special characters"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("topic_name", topic_name);
}

InvalidServiceNameError::InvalidServiceNameError(
  const std::string & service_name,
  const std::string & reason,
  const SourceLocation & location)
: ServiceException(
    ErrorCode::SERVICE_NAME_INVALID,
    "Invalid service name '" + service_name + "'" + (reason.empty() ? "" : ": " + reason),
    ErrorSuggestion(
      "Service names follow the same rules as topic names",
      {"Use absolute service names starting with '/'", "Avoid special characters"},
      "https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Names.html"
    ),
    location)
{
  with_context("service_name", service_name);
}

ServiceNotAvailableError::ServiceNotAvailableError(
  const std::string & service_name,
  const SourceLocation & location)
: ServiceException(
    ErrorCode::SERVICE_NOT_AVAILABLE,
    "Service '" + service_name + "' is not available",
    ErrorSuggestion(
      "Ensure the service server is running and properly advertised",
      {
        "Check if the service server node is running",
        "Use 'ros2 service list' to see available services",
        "Verify the service name matches exactly"
      }
    ),
    location)
{
  with_context("service_name", service_name);
}

ParameterNotDeclaredException::ParameterNotDeclaredException(
  const std::string & parameter_name,
  const SourceLocation & location)
: ParameterException(
    ErrorCode::PARAMETER_NOT_DECLARED,
    "Parameter '" + parameter_name + "' has not been declared",
    ErrorSuggestion(
      "Declare the parameter before use with declare_parameter<T>(name, default_value)",
      {
        "Use declare_parameter() in the node constructor",
        "Set allow_undeclared_parameters to true in NodeOptions",
        "Use has_parameter() to check existence first"
      },
      "https://docs.ros.org/en/rolling/Tutorials/Parameters/Understanding-ROS2-Parameters.html"
    ),
    location)
{
  with_context("parameter_name", parameter_name);
}

ParameterTypeMismatchException::ParameterTypeMismatchException(
  const std::string & parameter_name,
  const std::string & expected_type,
  const std::string & actual_type,
  const SourceLocation & location)
: ParameterException(
    ErrorCode::PARAMETER_TYPE_MISMATCH,
    "Parameter '" + parameter_name + "' type mismatch: expected " + expected_type +
    ", got " + actual_type,
    ErrorSuggestion(
      "Use the correct type when getting the parameter value",
      {
        "Check parameter declaration for the expected type",
        "Use get_parameter_or() for type-safe access"
      }
    ),
    location)
{
  with_context("parameter_name", parameter_name);
  with_context("expected_type", expected_type);
  with_context("actual_type", actual_type);
}

InvalidParameterValueException::InvalidParameterValueException(
  const std::string & parameter_name,
  const std::string & reason,
  const SourceLocation & location)
: ParameterException(
    ErrorCode::PARAMETER_INVALID_VALUE,
    "Invalid value for parameter '" + parameter_name + "': " + reason,
    ErrorSuggestion(
      "Provide a valid value that meets the parameter's constraints",
      {
        "Check parameter description for valid values",
        "Use describe_parameter() to see constraints"
      }
    ),
    location)
{
  with_context("parameter_name", parameter_name);
}

QoSIncompatibleError::QoSIncompatibleError(
  const std::string & topic_name,
  const std::string & publisher_qos,
  const std::string & subscriber_qos,
  const SourceLocation & location)
: QoSException(
    ErrorCode::QOS_INCOMPATIBLE,
    "QoS incompatibility on topic '" + topic_name + "'",
    ErrorSuggestion(
      "Ensure publisher and subscriber QoS settings are compatible",
      {
        "Use matching reliability settings (reliable/best_effort)",
        "Use compatible durability settings",
        "Consider using QoS profiles like rclcpp::SensorDataQoS() for consistent settings"
      },
      "https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html"
    ),
    location)
{
  with_context("topic_name", topic_name);
  with_context("publisher_qos", publisher_qos);
  with_context("subscriber_qos", subscriber_qos);
}

CallbackGroupError::CallbackGroupError(
  const std::string & reason,
  const SourceLocation & location)
: ExecutorException(
    ErrorCode::CALLBACK_GROUP_ERROR,
    "Callback group error: " + reason,
    ErrorSuggestion(
      "Review callback group configuration and ensure proper assignment",
      {
        "Check that callbacks are added to appropriate groups",
        "Verify executor handles the callback group type (mutually exclusive vs reentrant)"
      }
    ),
    location)
{
}
#endif

}  // namespace exceptions
}  // namespace rclcpp
