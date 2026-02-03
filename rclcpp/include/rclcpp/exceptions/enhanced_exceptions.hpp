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

#ifndef RCLCPP__EXCEPTIONS__ENHANCED_EXCEPTIONS_HPP_
#define RCLCPP__EXCEPTIONS__ENHANCED_EXCEPTIONS_HPP_

#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/exceptions/error_codes.hpp"
#include "rclcpp/visibility_control.hpp"

// Use source_location if available (C++20), otherwise provide fallback
#if __cplusplus >= 202002L && __has_include(<source_location>)
#include <source_location>
#define RCLCPP_HAS_SOURCE_LOCATION 1
#else
#define RCLCPP_HAS_SOURCE_LOCATION 0
#endif

namespace rclcpp
{
namespace exceptions
{

#if !RCLCPP_HAS_SOURCE_LOCATION
/// Fallback source location for pre-C++20 compilers
struct SourceLocation
{
  const char * file_name_ = "";
  const char * function_name_ = "";
  uint32_t line_ = 0;
  uint32_t column_ = 0;

  constexpr const char * file_name() const noexcept {return file_name_;}
  constexpr const char * function_name() const noexcept {return function_name_;}
  constexpr uint32_t line() const noexcept {return line_;}
  constexpr uint32_t column() const noexcept {return column_;}

  static constexpr SourceLocation current(
    const char * file = __builtin_FILE(),
    const char * func = __builtin_FUNCTION(),
    uint32_t line = __builtin_LINE()) noexcept
  {
    return SourceLocation{file, func, line, 0};
  }
};
#else
using SourceLocation = std::source_location;
#endif

/// Forward declaration for context builder
class ErrorContext;

/// Structure to hold recovery suggestions
struct RCLCPP_PUBLIC ErrorSuggestion
{
  std::string primary;                      ///< Main suggestion for fixing the error
  std::vector<std::string> alternatives;    ///< Alternative solutions
  std::string documentation_url;            ///< Link to relevant documentation

  /// Create an empty suggestion
  ErrorSuggestion() = default;

  /// Create a suggestion with just a primary message
  explicit ErrorSuggestion(const std::string & primary_suggestion)
  : primary(primary_suggestion) {}

  /// Create a suggestion with primary and alternatives
  ErrorSuggestion(
    const std::string & primary_suggestion,
    const std::vector<std::string> & alt_suggestions)
  : primary(primary_suggestion), alternatives(alt_suggestions) {}

  /// Create a full suggestion with documentation link
  ErrorSuggestion(
    const std::string & primary_suggestion,
    const std::vector<std::string> & alt_suggestions,
    const std::string & doc_url)
  : primary(primary_suggestion), alternatives(alt_suggestions), documentation_url(doc_url) {}

  /// Check if the suggestion is empty
  [[nodiscard]] bool empty() const noexcept
  {
    return primary.empty() && alternatives.empty();
  }

  /// Format the suggestion as a string
  [[nodiscard]] std::string format() const;
};

/// Base exception class for all rclcpp exceptions with enhanced information
/**
 * This exception class provides:
 * - Error codes for programmatic handling
 * - Contextual information about the error
 * - Recovery suggestions
 * - Source location information
 * - Optional stack trace support
 *
 * It maintains backward compatibility with std::exception by providing
 * a meaningful what() message.
 */
class RCLCPP_PUBLIC RclcppException : public std::exception
{
public:
  /// Construct an exception with an error code and message
  /**
   * \param[in] code The error code identifying the type of error
   * \param[in] message A human-readable description of the error
   * \param[in] location Source location where the exception was thrown
   */
  explicit RclcppException(
    ErrorCode code,
    const std::string & message,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );

  /// Construct an exception with error code, message, and suggestion
  /**
   * \param[in] code The error code identifying the type of error
   * \param[in] message A human-readable description of the error
   * \param[in] suggestion A suggestion for how to fix the error
   * \param[in] location Source location where the exception was thrown
   */
  RclcppException(
    ErrorCode code,
    const std::string & message,
    const std::string & suggestion,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );

  /// Construct an exception with full error suggestion
  /**
   * \param[in] code The error code identifying the type of error
   * \param[in] message A human-readable description of the error
   * \param[in] suggestion Full suggestion structure with alternatives
   * \param[in] location Source location where the exception was thrown
   */
  RclcppException(
    ErrorCode code,
    const std::string & message,
    const ErrorSuggestion & suggestion,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );

  /// Construct an exception with context
  /**
   * \param[in] code The error code identifying the type of error
   * \param[in] message A human-readable description of the error
   * \param[in] suggestion A suggestion for how to fix the error
   * \param[in] context Additional contextual information
   * \param[in] location Source location where the exception was thrown
   */
  RclcppException(
    ErrorCode code,
    const std::string & message,
    const ErrorSuggestion & suggestion,
    const ErrorContext & context,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );

  /// Copy constructor
  RclcppException(const RclcppException & other);

  /// Move constructor
  RclcppException(RclcppException && other) noexcept;

  /// Copy assignment operator
  RclcppException & operator=(const RclcppException & other);

  /// Move assignment operator
  RclcppException & operator=(RclcppException && other) noexcept;

  /// Destructor
  ~RclcppException() override;

  /// Get the error code
  /**
   * \return The error code associated with this exception
   */
  [[nodiscard]] ErrorCode error_code() const noexcept;

  /// Get the error category
  /**
   * \return The category of this error
   */
  [[nodiscard]] ErrorCategory error_category() const noexcept;

  /// Get the error severity
  /**
   * \return The severity level of this error
   */
  [[nodiscard]] ErrorSeverity error_severity() const noexcept;

  /// Get the error message (without formatting)
  /**
   * \return The raw error message
   */
  [[nodiscard]] const std::string & message() const noexcept;

  /// Get the primary suggestion for fixing the error
  /**
   * \return The suggestion string, or empty if none provided
   */
  [[nodiscard]] const std::string & suggestion() const noexcept;

  /// Get the full suggestion structure
  /**
   * \return The complete suggestion with alternatives
   */
  [[nodiscard]] const ErrorSuggestion & full_suggestion() const noexcept;

  /// Get the formatted context information
  /**
   * \return Context as a formatted string
   */
  [[nodiscard]] std::string context() const;

  /// Get the context as a key-value map
  /**
   * \return Map of context key-value pairs
   */
  [[nodiscard]] const std::map<std::string, std::string> & context_map() const noexcept;

  /// Get the source file name where the exception was thrown
  /**
   * \return File name or empty string if not available
   */
  [[nodiscard]] const char * file_name() const noexcept;

  /// Get the function name where the exception was thrown
  /**
   * \return Function name or empty string if not available
   */
  [[nodiscard]] const char * function_name() const noexcept;

  /// Get the line number where the exception was thrown
  /**
   * \return Line number or 0 if not available
   */
  [[nodiscard]] uint32_t line() const noexcept;

  /// Get the formatted location string
  /**
   * \return Location as "file:line in function()"
   */
  [[nodiscard]] std::string location_string() const;

  /// Get the full error message with all available information
  /**
   * This is the standard exception interface. Returns a formatted
   * message including error code, message, context, and suggestion.
   *
   * \return Formatted error message
   */
  [[nodiscard]] const char * what() const noexcept override;

  /// Get a detailed multi-line error report
  /**
   * \return Detailed error information formatted for logging
   */
  [[nodiscard]] std::string detailed_report() const;

  /// Add context information to the exception
  /**
   * \param[in] key Context key
   * \param[in] value Context value
   * \return Reference to this exception for chaining
   */
  RclcppException & with_context(const std::string & key, const std::string & value);

protected:
  /// Format the what() message - called lazily
  void format_what_message() const;

private:
  /// Private implementation to allow future changes without ABI breaks
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Builder class for constructing error context
/**
 * Provides a fluent interface for building up error context information
 * that can be attached to exceptions.
 */
class RCLCPP_PUBLIC ErrorContext
{
public:
  /// Default constructor
  ErrorContext() = default;

  /// Copy constructor
  ErrorContext(const ErrorContext & other);

  /// Move constructor
  ErrorContext(ErrorContext && other) noexcept;

  /// Copy assignment
  ErrorContext & operator=(const ErrorContext & other);

  /// Move assignment
  ErrorContext & operator=(ErrorContext && other) noexcept;

  /// Destructor
  ~ErrorContext();

  /// Add node name to context
  ErrorContext & with_node_name(const std::string & name);

  /// Add topic name to context
  ErrorContext & with_topic_name(const std::string & name);

  /// Add service name to context
  ErrorContext & with_service_name(const std::string & name);

  /// Add parameter name to context
  ErrorContext & with_parameter_name(const std::string & name);

  /// Add expected type information
  ErrorContext & with_expected_type(const std::string & type);

  /// Add actual type information
  ErrorContext & with_actual_type(const std::string & type);

  /// Add expected value information
  ErrorContext & with_expected_value(const std::string & value);

  /// Add actual value information
  ErrorContext & with_actual_value(const std::string & value);

  /// Add QoS profile information
  ErrorContext & with_qos_profile(const std::string & profile);

  /// Add namespace information
  ErrorContext & with_namespace(const std::string & ns);

  /// Add custom key-value pair
  ErrorContext & with(const std::string & key, const std::string & value);

  /// Format the context as a string
  [[nodiscard]] std::string format() const;

  /// Get the context as a map
  [[nodiscard]] const std::map<std::string, std::string> & as_map() const noexcept;

  /// Check if context is empty
  [[nodiscard]] bool empty() const noexcept;

private:
  std::map<std::string, std::string> context_;
};

// ============================================================================
// Specialized Exception Classes
// ============================================================================

/// Base class for node-related exceptions
class RCLCPP_PUBLIC NodeException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown when a node name is invalid
class RCLCPP_PUBLIC InvalidNodeNameError : public NodeException
{
public:
  explicit InvalidNodeNameError(
    const std::string & node_name,
    const std::string & reason = "",
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Exception thrown when a namespace is invalid
class RCLCPP_PUBLIC InvalidNamespaceError : public NodeException
{
public:
  explicit InvalidNamespaceError(
    const std::string & namespace_name,
    const std::string & reason = "",
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Exception thrown when node initialization fails
class RCLCPP_PUBLIC NodeInitializationError : public NodeException
{
public:
  explicit NodeInitializationError(
    const std::string & node_name,
    const std::string & reason,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Base class for topic-related exceptions
class RCLCPP_PUBLIC TopicException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown when a topic name is invalid
class RCLCPP_PUBLIC InvalidTopicNameError : public TopicException
{
public:
  explicit InvalidTopicNameError(
    const std::string & topic_name,
    const std::string & reason = "",
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Base class for service-related exceptions
class RCLCPP_PUBLIC ServiceException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown when a service name is invalid
class RCLCPP_PUBLIC InvalidServiceNameError : public ServiceException
{
public:
  explicit InvalidServiceNameError(
    const std::string & service_name,
    const std::string & reason = "",
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Exception thrown when a service is not available
class RCLCPP_PUBLIC ServiceNotAvailableError : public ServiceException
{
public:
  explicit ServiceNotAvailableError(
    const std::string & service_name,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Base class for parameter-related exceptions
class RCLCPP_PUBLIC ParameterException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown when accessing an undeclared parameter
class RCLCPP_PUBLIC ParameterNotDeclaredException : public ParameterException
{
public:
  explicit ParameterNotDeclaredException(
    const std::string & parameter_name,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Exception thrown when parameter type doesn't match
class RCLCPP_PUBLIC ParameterTypeMismatchException : public ParameterException
{
public:
  ParameterTypeMismatchException(
    const std::string & parameter_name,
    const std::string & expected_type,
    const std::string & actual_type,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Exception thrown when parameter value is invalid
class RCLCPP_PUBLIC InvalidParameterValueException : public ParameterException
{
public:
  InvalidParameterValueException(
    const std::string & parameter_name,
    const std::string & reason,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Base class for QoS-related exceptions
class RCLCPP_PUBLIC QoSException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown when QoS settings are incompatible
class RCLCPP_PUBLIC QoSIncompatibleError : public QoSException
{
public:
  QoSIncompatibleError(
    const std::string & topic_name,
    const std::string & publisher_qos,
    const std::string & subscriber_qos,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

/// Base class for executor-related exceptions
class RCLCPP_PUBLIC ExecutorException : public RclcppException
{
public:
  using RclcppException::RclcppException;
};

/// Exception thrown for callback group configuration errors
class RCLCPP_PUBLIC CallbackGroupError : public ExecutorException
{
public:
  explicit CallbackGroupError(
    const std::string & reason,
#if RCLCPP_HAS_SOURCE_LOCATION
    const std::source_location & location = std::source_location::current()
#else
    const SourceLocation & location = SourceLocation::current()
#endif
  );
};

}  // namespace exceptions
}  // namespace rclcpp

#endif  // RCLCPP__EXCEPTIONS__ENHANCED_EXCEPTIONS_HPP_
