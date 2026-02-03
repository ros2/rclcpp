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

#ifndef RCLCPP__PARAMETER_VALIDATION_HPP_
#define RCLCPP__PARAMETER_VALIDATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace rclcpp
{

/// \brief Result of a parameter validation operation.
///
/// This class encapsulates the outcome of validating a parameter value,
/// including whether the validation succeeded and a reason message if it failed.
class RCLCPP_PUBLIC ValidationResult
{
public:
  /// \brief Create a successful validation result.
  /// \return A ValidationResult indicating success.
  static ValidationResult success();

  /// \brief Create a failed validation result.
  /// \param reason A human-readable description of why validation failed.
  /// \return A ValidationResult indicating failure with the given reason.
  static ValidationResult failure(const std::string & reason);

  /// \brief Check if the validation was successful.
  /// \return true if validation succeeded, false otherwise.
  [[nodiscard]] bool
  successful() const noexcept;

  /// \brief Boolean conversion operator.
  /// \return true if validation succeeded, false otherwise.
  [[nodiscard]] explicit
  operator bool() const noexcept;

  /// \brief Get the failure reason.
  /// \return The reason string if validation failed, empty string if successful.
  [[nodiscard]] const std::string &
  reason() const noexcept;

  /// \brief Convert to ROS message type for use with parameter callbacks.
  /// \return A SetParametersResult message representing this validation result.
  [[nodiscard]] rcl_interfaces::msg::SetParametersResult
  to_msg() const;

private:
  ValidationResult(bool successful, std::string reason);

  bool successful_;
  std::string reason_;
};

/// \brief Abstract base class for parameter validators.
///
/// This class defines the interface that all parameter validators must implement.
/// Validators are used to check if a parameter value meets certain constraints.
///
/// To create a custom validator, inherit from this class and implement the
/// validate() and description() methods.
///
/// Example:
/// \code{.cpp}
/// class PositiveValidator : public rclcpp::ParameterValidator {
/// public:
///   ValidationResult validate(const rclcpp::Parameter& param) const override {
///     if (param.as_double() > 0.0) {
///       return ValidationResult::success();
///     }
///     return ValidationResult::failure("Value must be positive");
///   }
///
///   std::string description() const override {
///     return "positive number";
///   }
/// };
/// \endcode
class RCLCPP_PUBLIC ParameterValidator
{
public:
  /// \brief Virtual destructor.
  virtual ~ParameterValidator() = default;

  /// \brief Validate a parameter value.
  /// \param param The parameter to validate.
  /// \return A ValidationResult indicating success or failure.
  [[nodiscard]] virtual ValidationResult
  validate(const rclcpp::Parameter & param) const = 0;

  /// \brief Get a human-readable description of the validation constraint.
  /// \return A string describing what this validator checks for.
  [[nodiscard]] virtual std::string
  description() const = 0;

  /// \brief Get the expected parameter types for this validator.
  ///
  /// By default, returns an empty vector indicating any type is accepted.
  /// Override this method to restrict which parameter types are valid.
  ///
  /// \return A vector of acceptable ParameterType values.
  [[nodiscard]] virtual std::vector<rclcpp::ParameterType>
  expected_types() const;
};

/// \brief Shared pointer type for ParameterValidator.
using ParameterValidatorPtr = std::shared_ptr<ParameterValidator>;

/// \brief Shared pointer to const type for ParameterValidator.
using ParameterValidatorConstPtr = std::shared_ptr<const ParameterValidator>;

/// \brief Format a validation error message with consistent structure.
///
/// Creates an error message in the format:
/// "Parameter '<name>' validation failed: <reason>
///   Expected: <expected>
///   Got: <actual>"
///
/// \param param_name The name of the parameter that failed validation.
/// \param reason Brief description of why validation failed.
/// \param expected Description of what was expected.
/// \param actual The actual value that was provided.
/// \return A formatted error message string.
RCLCPP_PUBLIC
std::string
format_validation_error(
  const std::string & param_name,
  const std::string & reason,
  const std::string & expected,
  const std::string & actual);

/// \brief Convert a parameter value to string for error messages.
/// \param param The parameter to convert.
/// \return A string representation of the parameter value.
RCLCPP_PUBLIC
std::string
parameter_value_to_string(const rclcpp::Parameter & param);

/// \brief Get the string name of a parameter type.
/// \param type The parameter type.
/// \return A string name for the type.
RCLCPP_PUBLIC
std::string
parameter_type_to_string(rclcpp::ParameterType type);

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_VALIDATION_HPP_
