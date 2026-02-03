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

#ifndef RCLCPP__PARAMETER_BUILDER_HPP_
#define RCLCPP__PARAMETER_BUILDER_HPP_

#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_validation.hpp"
#include "rclcpp/parameter_validators.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace rclcpp
{

/// \brief Fluent builder for creating validated parameters.
///
/// ParameterBuilder provides a fluent API for declaring parameters with
/// validation constraints. It allows you to specify the parameter name,
/// description, default value, and one or more validators.
///
/// Example:
/// \code{.cpp}
/// auto builder = ParameterBuilder()
///   .name("max_velocity")
///   .description("Maximum robot velocity in m/s")
///   .default_value(1.0)
///   .range(0.0, 10.0);
///
/// // Declare the parameter with validation
/// declare_validated_parameter(node, builder);
/// \endcode
class RCLCPP_PUBLIC ParameterBuilder
{
public:
  /// \brief Default constructor.
  ParameterBuilder() = default;

  /// \brief Construct with parameter name.
  /// \param param_name The name of the parameter.
  explicit ParameterBuilder(const std::string & param_name);

  /// \brief Set the parameter name.
  /// \param param_name The name of the parameter.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & name(const std::string & param_name);

  /// \brief Set the parameter description.
  /// \param desc Human-readable description of the parameter.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & description(const std::string & desc);

  /// \brief Set whether the parameter is read-only.
  /// \param is_read_only True if parameter should be read-only.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & read_only(bool is_read_only = true);

  /// \brief Set the default value (template version).
  /// \tparam T The value type.
  /// \param value The default value.
  /// \return Reference to this builder for chaining.
  template<typename T>
  ParameterBuilder & default_value(const T & value)
  {
    default_value_ = rclcpp::ParameterValue(value);
    return *this;
  }

  /// \brief Set the default value from ParameterValue.
  /// \param value The default value.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & default_value(const rclcpp::ParameterValue & value);

  /// \brief Add a custom validator.
  /// \param v The validator to add.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & validator(ParameterValidatorPtr v);

  /// \brief Add a range validator for numeric parameters.
  /// \tparam T The numeric type (int64_t or double).
  /// \param min_val Minimum allowed value (inclusive).
  /// \param max_val Maximum allowed value (inclusive).
  /// \return Reference to this builder for chaining.
  template<typename T>
  ParameterBuilder & range(T min_val, T max_val)
  {
    validators_.push_back(std::make_shared<RangeValidator<T>>(min_val, max_val));
    return *this;
  }

  /// \brief Add a minimum value validator.
  /// \tparam T The numeric type.
  /// \param min_val Minimum allowed value (inclusive).
  /// \return Reference to this builder for chaining.
  template<typename T>
  ParameterBuilder & min(T min_val)
  {
    validators_.push_back(
      std::make_shared<RangeValidator<T>>(RangeValidator<T>::min(min_val)));
    return *this;
  }

  /// \brief Add a maximum value validator.
  /// \tparam T The numeric type.
  /// \param max_val Maximum allowed value (inclusive).
  /// \return Reference to this builder for chaining.
  template<typename T>
  ParameterBuilder & max(T max_val)
  {
    validators_.push_back(
      std::make_shared<RangeValidator<T>>(RangeValidator<T>::max(max_val)));
    return *this;
  }

  /// \brief Add a one-of validator for enum-like constraints.
  /// \tparam T The value type.
  /// \param values The allowed values.
  /// \return Reference to this builder for chaining.
  template<typename T>
  ParameterBuilder & one_of(std::initializer_list<T> values)
  {
    validators_.push_back(std::make_shared<OneOfValidator<T>>(values));
    return *this;
  }

  /// \brief Add a regex pattern validator for strings.
  /// \param pattern The regex pattern to match.
  /// \param pattern_desc Optional description of the pattern.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & matches(
    const std::string & pattern,
    const std::string & pattern_desc = "");

  /// \brief Add a not-empty validator.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & not_empty();

  /// \brief Add a string length validator.
  /// \param min_len Minimum string length.
  /// \param max_len Maximum string length.
  /// \return Reference to this builder for chaining.
  ParameterBuilder & length(size_t min_len, size_t max_len);

  /// \brief Get the parameter name.
  /// \return The parameter name.
  [[nodiscard]] const std::string &
  get_name() const;

  /// \brief Get the parameter description.
  /// \return The description string.
  [[nodiscard]] const std::string &
  get_description() const;

  /// \brief Check if parameter is read-only.
  /// \return True if read-only.
  [[nodiscard]] bool
  is_read_only() const;

  /// \brief Check if a default value is set.
  /// \return True if default value is set.
  [[nodiscard]] bool
  has_default_value() const;

  /// \brief Get the default value.
  /// \return The default ParameterValue.
  /// \throws std::runtime_error if no default value is set.
  [[nodiscard]] const rclcpp::ParameterValue &
  get_default_value() const;

  /// \brief Build the parameter with current settings.
  /// \return The constructed Parameter.
  /// \throws std::runtime_error if name is not set.
  [[nodiscard]] rclcpp::Parameter
  build() const;

  /// \brief Build a ParameterDescriptor with current settings.
  ///
  /// This creates a descriptor that includes the description, read-only flag,
  /// and validation constraints as additional_constraints text.
  ///
  /// \return The constructed ParameterDescriptor.
  [[nodiscard]] rcl_interfaces::msg::ParameterDescriptor
  build_descriptor() const;

  /// \brief Get the list of validators.
  /// \return Vector of validator pointers.
  [[nodiscard]] const std::vector<ParameterValidatorPtr> &
  get_validators() const;

  /// \brief Validate a parameter value against all validators.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or first failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const;

private:
  std::string name_;
  std::string description_;
  bool read_only_ = false;
  std::optional<rclcpp::ParameterValue> default_value_;
  std::vector<ParameterValidatorPtr> validators_;
};

/// \brief Thread-safe registry for parameter validators.
///
/// This class manages the association between parameter names and their
/// validators, allowing automatic validation when parameters change.
class RCLCPP_PUBLIC ParameterValidatorRegistry
{
public:
  /// \brief Default constructor.
  ParameterValidatorRegistry() = default;

  /// \brief Register validators for a parameter.
  /// \param param_name The parameter name.
  /// \param validators Vector of validators to register.
  void register_validators(
    const std::string & param_name,
    std::vector<ParameterValidatorPtr> validators);

  /// \brief Add a validator for a parameter.
  /// \param param_name The parameter name.
  /// \param validator The validator to add.
  void add_validator(
    const std::string & param_name,
    ParameterValidatorPtr validator);

  /// \brief Remove all validators for a parameter.
  /// \param param_name The parameter name.
  void clear_validators(const std::string & param_name);

  /// \brief Check if a parameter has validators.
  /// \param param_name The parameter name.
  /// \return True if validators are registered.
  [[nodiscard]] bool
  has_validators(const std::string & param_name) const;

  /// \brief Validate a parameter against its registered validators.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or first failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const;

  /// \brief Validate multiple parameters.
  /// \param params Vector of parameters to validate.
  /// \return ValidationResult indicating success or first failure.
  [[nodiscard]] ValidationResult
  validate(const std::vector<rclcpp::Parameter> & params) const;

  /// \brief Create a parameter callback function for validation.
  ///
  /// Returns a callback suitable for use with add_on_set_parameters_callback().
  ///
  /// \return A callback function that validates parameters.
  [[nodiscard]] std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)>
  create_callback() const;

private:
  mutable std::mutex mutex_;
  std::map<std::string, std::vector<ParameterValidatorPtr>> validators_;
};

// =============================================================================
// Free Functions for Node Integration
// =============================================================================

/// \brief Declare a parameter with validation using ParameterBuilder.
///
/// This function declares a parameter on a node, registering the validators
/// from the builder and setting up automatic validation on parameter changes.
///
/// Example:
/// \code{.cpp}
/// auto param = declare_validated_parameter(
///   *this,
///   ParameterBuilder()
///     .name("max_velocity")
///     .description("Maximum velocity in m/s")
///     .default_value(1.0)
///     .range(0.0, 10.0)
/// );
/// \endcode
///
/// \tparam NodeT The node type (must have declare_parameter and
///         add_on_set_parameters_callback methods).
/// \param node The node to declare the parameter on.
/// \param builder The ParameterBuilder with parameter configuration.
/// \return The declared parameter value.
/// \throws rclcpp::exceptions::ParameterAlreadyDeclaredException if parameter
///         is already declared.
/// \throws rclcpp::exceptions::InvalidParameterValueException if the default
///         value fails validation.
template<typename NodeT>
rclcpp::Parameter
declare_validated_parameter(
  NodeT & node,
  const ParameterBuilder & builder)
{
  // Validate default value first
  if (builder.has_default_value()) {
    rclcpp::Parameter temp_param(builder.get_name(), builder.get_default_value());
    auto result = builder.validate(temp_param);
    if (!result) {
      throw rclcpp::exceptions::InvalidParameterValueException(result.reason());
    }
  }

  // Build descriptor
  auto descriptor = builder.build_descriptor();

  // Declare the parameter
  rclcpp::Parameter declared_param;
  if (builder.has_default_value()) {
    declared_param = rclcpp::Parameter(
      builder.get_name(),
      node.declare_parameter(
        builder.get_name(),
        builder.get_default_value(),
        descriptor));
  } else {
    // Declare without default - will be PARAMETER_NOT_SET
    declared_param = rclcpp::Parameter(
      builder.get_name(),
      node.declare_parameter(builder.get_name(), rclcpp::ParameterValue(), descriptor));
  }

  // Register validation callback if there are validators
  if (!builder.get_validators().empty()) {
    auto validators = builder.get_validators();
    std::string param_name = builder.get_name();

    auto callback = [validators, param_name](
      const std::vector<rclcpp::Parameter> & params)
      -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : params) {
          if (param.get_name() == param_name) {
            for (const auto & validator : validators) {
              auto validation = validator->validate(param);
              if (!validation) {
                result.successful = false;
                result.reason = validation.reason();
                return result;
              }
            }
          }
        }

        return result;
      };

    node.add_on_set_parameters_callback(callback);
  }

  return declared_param;
}

/// \brief Declare multiple validated parameters from a list of builders.
///
/// This is more efficient than calling declare_validated_parameter() multiple
/// times because it creates a single combined callback for all validators.
///
/// \tparam NodeT The node type.
/// \param node The node to declare parameters on.
/// \param builders Vector of ParameterBuilder objects.
/// \return Vector of declared parameters.
template<typename NodeT>
std::vector<rclcpp::Parameter>
declare_validated_parameters(
  NodeT & node,
  const std::vector<ParameterBuilder> & builders)
{
  std::vector<rclcpp::Parameter> declared_params;
  declared_params.reserve(builders.size());

  // Collect all validators
  std::map<std::string, std::vector<ParameterValidatorPtr>> all_validators;

  for (const auto & builder : builders) {
    // Validate default value first
    if (builder.has_default_value()) {
      rclcpp::Parameter temp_param(builder.get_name(), builder.get_default_value());
      auto result = builder.validate(temp_param);
      if (!result) {
        throw rclcpp::exceptions::InvalidParameterValueException(result.reason());
      }
    }

    // Build descriptor
    auto descriptor = builder.build_descriptor();

    // Declare the parameter
    rclcpp::Parameter declared_param;
    if (builder.has_default_value()) {
      declared_param = rclcpp::Parameter(
        builder.get_name(),
        node.declare_parameter(
          builder.get_name(),
          builder.get_default_value(),
          descriptor));
    } else {
      declared_param = rclcpp::Parameter(
        builder.get_name(),
        node.declare_parameter(builder.get_name(), rclcpp::ParameterValue(), descriptor));
    }

    declared_params.push_back(declared_param);

    // Collect validators
    if (!builder.get_validators().empty()) {
      all_validators[builder.get_name()] = builder.get_validators();
    }
  }

  // Register single combined callback
  if (!all_validators.empty()) {
    auto callback = [all_validators](
      const std::vector<rclcpp::Parameter> & params)
      -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : params) {
          auto it = all_validators.find(param.get_name());
          if (it != all_validators.end()) {
            for (const auto & validator : it->second) {
              auto validation = validator->validate(param);
              if (!validation) {
                result.successful = false;
                result.reason = validation.reason();
                return result;
              }
            }
          }
        }

        return result;
      };

    node.add_on_set_parameters_callback(callback);
  }

  return declared_params;
}

/// \brief Create a validation callback for pre-existing parameters.
///
/// Use this when you need to add validation to parameters that were already
/// declared through other means.
///
/// Example:
/// \code{.cpp}
/// std::map<std::string, std::vector<ParameterValidatorPtr>> validators;
/// validators["velocity"] = {validators::range(0.0, 10.0)};
/// validators["mode"] = {validators::one_of({"slow", "fast"})};
///
/// auto handle = create_validation_callback(node, validators);
/// \endcode
///
/// \tparam NodeT The node type.
/// \param node The node to register the callback on.
/// \param validators Map of parameter names to their validators.
/// \return The callback handle (store this to keep the callback active).
template<typename NodeT>
typename rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
create_validation_callback(
  NodeT & node,
  const std::map<std::string, std::vector<ParameterValidatorPtr>> & validators)
{
  auto callback = [validators](
    const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto & param : params) {
        auto it = validators.find(param.get_name());
        if (it != validators.end()) {
          for (const auto & validator : it->second) {
            auto validation = validator->validate(param);
            if (!validation) {
              result.successful = false;
              result.reason = validation.reason();
              return result;
            }
          }
        }
      }

      return result;
    };

  return node.add_on_set_parameters_callback(callback);
}

/// \brief Validate a parameter value programmatically.
///
/// This is useful for validating values before attempting to set them,
/// or for validating values in custom logic.
///
/// \param param The parameter to validate.
/// \param validators Vector of validators to apply.
/// \return ValidationResult indicating success or first failure.
RCLCPP_PUBLIC
ValidationResult
validate_parameter(
  const rclcpp::Parameter & param,
  const std::vector<ParameterValidatorPtr> & validators);

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_BUILDER_HPP_
