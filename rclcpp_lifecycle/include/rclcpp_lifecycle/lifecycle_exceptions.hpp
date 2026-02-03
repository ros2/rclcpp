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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_EXCEPTIONS_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"

namespace rclcpp_lifecycle_enhanced
{

/**
 * @brief Enumeration of possible transition error codes
 *
 * This enum provides detailed error codes for lifecycle transition failures,
 * enabling more precise error handling and recovery strategies.
 */
enum class TransitionErrorCode
{
  /// No error occurred
  NONE = 0,

  /// The requested transition is not valid from the current state
  INVALID_TRANSITION = 1,

  /// The transition callback returned a failure status
  CALLBACK_FAILURE = 2,

  /// The transition timed out
  TIMEOUT = 3,

  /// An internal error occurred during the transition
  INTERNAL_ERROR = 4,

  /// The node is in an error state and cannot transition
  ERROR_STATE = 5,

  /// A precondition for the transition was not met
  PRECONDITION_FAILED = 6,

  /// The transition was cancelled
  CANCELLED = 7,

  /// An unknown error occurred
  UNKNOWN = 99
};

/**
 * @brief Convert a TransitionErrorCode to its string representation
 * @param code The error code to convert
 * @return String representation of the error code
 */
inline std::string transition_error_code_to_string(TransitionErrorCode code)
{
  switch (code) {
    case TransitionErrorCode::NONE:
      return "NONE";
    case TransitionErrorCode::INVALID_TRANSITION:
      return "INVALID_TRANSITION";
    case TransitionErrorCode::CALLBACK_FAILURE:
      return "CALLBACK_FAILURE";
    case TransitionErrorCode::TIMEOUT:
      return "TIMEOUT";
    case TransitionErrorCode::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case TransitionErrorCode::ERROR_STATE:
      return "ERROR_STATE";
    case TransitionErrorCode::PRECONDITION_FAILED:
      return "PRECONDITION_FAILED";
    case TransitionErrorCode::CANCELLED:
      return "CANCELLED";
    default:
      return "UNKNOWN";
  }
}

/**
 * @class LifecycleException
 * @brief Base exception class for lifecycle-related errors
 *
 * Provides a common base class for all lifecycle exceptions, enabling
 * catch blocks to handle any lifecycle error uniformly if desired.
 */
class LifecycleException : public std::runtime_error
{
public:
  /**
   * @brief Construct a LifecycleException with a message
   * @param message The error message
   */
  explicit LifecycleException(const std::string & message)
  : std::runtime_error(message)
  {
  }

  /**
   * @brief Virtual destructor
   */
  ~LifecycleException() override = default;
};

/**
 * @class LifecycleTransitionException
 * @brief Exception thrown when a lifecycle state transition fails
 *
 * This exception provides detailed information about transition failures,
 * including the error code, source and target states, and recovery suggestions.
 * It enables callers to implement sophisticated error handling and recovery
 * strategies based on the specific failure mode.
 *
 * Example usage:
 * @code{.cpp}
 * try {
 *   node->safe_transition(transition);
 * } catch (const LifecycleTransitionException & e) {
 *   RCLCPP_ERROR(logger, "Transition failed: %s", e.what());
 *   RCLCPP_INFO(logger, "Recovery suggestion: %s", e.recovery_suggestion().c_str());
 *   if (e.error_code() == TransitionErrorCode::TIMEOUT) {
 *     // Handle timeout specifically
 *   }
 * }
 * @endcode
 */
class LifecycleTransitionException : public LifecycleException
{
public:
  /**
   * @brief Construct a LifecycleTransitionException with full details
   *
   * @param message Human-readable error message
   * @param error_code Specific error code identifying the failure type
   * @param from_state The state the node was in when the transition was attempted
   * @param to_state The target state that was not reached
   * @param recovery_suggestion A suggestion for how to recover from this error
   */
  LifecycleTransitionException(
    const std::string & message,
    TransitionErrorCode error_code,
    const rclcpp_lifecycle::State & from_state,
    const rclcpp_lifecycle::State & to_state,
    const std::string & recovery_suggestion = "")
  : LifecycleException(build_message(message, error_code, from_state, to_state)),
    error_code_(error_code),
    from_state_(from_state),
    to_state_(to_state),
    recovery_suggestion_(recovery_suggestion)
  {
  }

  /**
   * @brief Construct a LifecycleTransitionException with minimal information
   *
   * @param message Human-readable error message
   * @param error_code Specific error code identifying the failure type
   */
  LifecycleTransitionException(
    const std::string & message,
    TransitionErrorCode error_code)
  : LifecycleException(message + " [" + transition_error_code_to_string(error_code) + "]"),
    error_code_(error_code),
    from_state_(rclcpp_lifecycle::State()),
    to_state_(rclcpp_lifecycle::State()),
    recovery_suggestion_("")
  {
  }

  /**
   * @brief Get the error code for this transition failure
   * @return The TransitionErrorCode indicating the type of failure
   */
  TransitionErrorCode error_code() const noexcept
  {
    return error_code_;
  }

  /**
   * @brief Get the state the node was in when the transition failed
   * @return The source state
   */
  const rclcpp_lifecycle::State & from_state() const noexcept
  {
    return from_state_;
  }

  /**
   * @brief Get the target state that was not reached
   * @return The target state
   */
  const rclcpp_lifecycle::State & to_state() const noexcept
  {
    return to_state_;
  }

  /**
   * @brief Get a suggestion for recovering from this error
   * @return A human-readable recovery suggestion, or empty string if none
   */
  const std::string & recovery_suggestion() const noexcept
  {
    return recovery_suggestion_;
  }

private:
  /**
   * @brief Build a detailed error message
   */
  static std::string build_message(
    const std::string & message,
    TransitionErrorCode error_code,
    const rclcpp_lifecycle::State & from_state,
    const rclcpp_lifecycle::State & to_state)
  {
    std::string full_message = message;
    full_message += " [" + transition_error_code_to_string(error_code) + "]";
    full_message += " (from '" + from_state.label() + "' to '" + to_state.label() + "')";
    return full_message;
  }

  TransitionErrorCode error_code_;
  rclcpp_lifecycle::State from_state_;
  rclcpp_lifecycle::State to_state_;
  std::string recovery_suggestion_;
};

/**
 * @class InvalidStateException
 * @brief Exception thrown when an operation is attempted in an invalid state
 *
 * This exception is thrown when a method is called that requires the node
 * to be in a specific state, but the node is not in that state.
 */
class InvalidStateException : public LifecycleException
{
public:
  /**
   * @brief Construct an InvalidStateException
   *
   * @param operation The operation that was attempted
   * @param current_state The current state of the node
   * @param required_state Description of the required state(s)
   */
  InvalidStateException(
    const std::string & operation,
    const rclcpp_lifecycle::State & current_state,
    const std::string & required_state)
  : LifecycleException(
      "Cannot perform operation '" + operation + "' in state '" +
      current_state.label() + "'. Required state: " + required_state),
    operation_(operation),
    current_state_(current_state),
    required_state_(required_state)
  {
  }

  /**
   * @brief Get the operation that was attempted
   * @return The operation name
   */
  const std::string & operation() const noexcept
  {
    return operation_;
  }

  /**
   * @brief Get the current state when the exception was thrown
   * @return The current state
   */
  const rclcpp_lifecycle::State & current_state() const noexcept
  {
    return current_state_;
  }

  /**
   * @brief Get the required state description
   * @return Description of required state(s)
   */
  const std::string & required_state() const noexcept
  {
    return required_state_;
  }

private:
  std::string operation_;
  rclcpp_lifecycle::State current_state_;
  std::string required_state_;
};

/**
 * @class TransitionTimeoutException
 * @brief Exception thrown when a transition times out
 *
 * This exception is a specialized form of LifecycleTransitionException
 * specifically for timeout scenarios, providing additional timeout information.
 */
class TransitionTimeoutException : public LifecycleTransitionException
{
public:
  /**
   * @brief Construct a TransitionTimeoutException
   *
   * @param transition_name The name of the transition that timed out
   * @param timeout_ms The timeout value in milliseconds
   * @param from_state The state the node was in when the transition was attempted
   * @param to_state The target state that was not reached
   */
  TransitionTimeoutException(
    const std::string & transition_name,
    int64_t timeout_ms,
    const rclcpp_lifecycle::State & from_state,
    const rclcpp_lifecycle::State & to_state)
  : LifecycleTransitionException(
      "Transition '" + transition_name + "' timed out after " +
      std::to_string(timeout_ms) + "ms",
      TransitionErrorCode::TIMEOUT,
      from_state,
      to_state,
      "Consider increasing the timeout or investigating why the transition callback is slow"),
    transition_name_(transition_name),
    timeout_ms_(timeout_ms)
  {
  }

  /**
   * @brief Get the name of the transition that timed out
   * @return The transition name
   */
  const std::string & transition_name() const noexcept
  {
    return transition_name_;
  }

  /**
   * @brief Get the timeout value in milliseconds
   * @return The timeout value
   */
  int64_t timeout_ms() const noexcept
  {
    return timeout_ms_;
  }

private:
  std::string transition_name_;
  int64_t timeout_ms_;
};

}  // namespace rclcpp_lifecycle_enhanced

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_EXCEPTIONS_HPP_
