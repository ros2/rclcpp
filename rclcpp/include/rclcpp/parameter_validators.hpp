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

#ifndef RCLCPP__PARAMETER_VALIDATORS_HPP_
#define RCLCPP__PARAMETER_VALIDATORS_HPP_

#include <algorithm>
#include <initializer_list>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "rclcpp/parameter_validation.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// \brief Validates that a numeric parameter is within a specified range.
///
/// This validator checks that integer or double parameter values fall within
/// specified minimum and/or maximum bounds. Bounds can be inclusive (default)
/// or exclusive.
///
/// Example:
/// \code{.cpp}
/// // Value must be in [0.0, 100.0]
/// auto validator = RangeValidator<double>(0.0, 100.0);
///
/// // Value must be in (0.0, 100.0) (exclusive bounds)
/// auto validator = RangeValidator<double>(0.0, 100.0).exclusive();
///
/// // Value must be >= 0 (no upper bound)
/// auto validator = RangeValidator<int64_t>::min(0);
///
/// // Value must be < 100 (exclusive upper bound only)
/// auto validator = RangeValidator<int64_t>::max(100).exclusive_max();
/// \endcode
///
/// \tparam T The numeric type (must be arithmetic: int64_t or double)
template<typename T>
class RangeValidator : public ParameterValidator
{
  static_assert(
    std::is_arithmetic_v<T>,
    "RangeValidator only supports arithmetic types (int64_t, double)"
  );

public:
  /// \brief Construct a range validator with both min and max bounds.
  /// \param min_value The minimum allowed value (inclusive by default).
  /// \param max_value The maximum allowed value (inclusive by default).
  RangeValidator(T min_value, T max_value)
  : min_value_(min_value), max_value_(max_value),
    min_exclusive_(false), max_exclusive_(false)
  {
  }

  /// \brief Create a validator with only a minimum bound.
  /// \param min_value The minimum allowed value.
  /// \return A RangeValidator with no upper bound.
  static RangeValidator<T> min(T min_value)
  {
    RangeValidator<T> v;
    v.min_value_ = min_value;
    return v;
  }

  /// \brief Create a validator with only a maximum bound.
  /// \param max_value The maximum allowed value.
  /// \return A RangeValidator with no lower bound.
  static RangeValidator<T> max(T max_value)
  {
    RangeValidator<T> v;
    v.max_value_ = max_value;
    return v;
  }

  /// \brief Make the minimum bound exclusive.
  /// \return Reference to this validator for chaining.
  RangeValidator<T> & exclusive_min()
  {
    min_exclusive_ = true;
    return *this;
  }

  /// \brief Make the maximum bound exclusive.
  /// \return Reference to this validator for chaining.
  RangeValidator<T> & exclusive_max()
  {
    max_exclusive_ = true;
    return *this;
  }

  /// \brief Make both bounds exclusive.
  /// \return Reference to this validator for chaining.
  RangeValidator<T> & exclusive()
  {
    min_exclusive_ = true;
    max_exclusive_ = true;
    return *this;
  }

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override
  {
    T value;

    // Extract value based on parameter type
    if constexpr (std::is_integral_v<T>) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "integer type",
            parameter_type_to_string(param.get_type())));
      }
      value = static_cast<T>(param.as_int());
    } else {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
        param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "numeric type (integer or double)",
            parameter_type_to_string(param.get_type())));
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<T>(param.as_int());
      } else {
        value = static_cast<T>(param.as_double());
      }
    }

    // Check minimum bound
    if (min_value_.has_value()) {
      bool min_ok = min_exclusive_ ?
        (value > min_value_.value()) :
        (value >= min_value_.value());
      if (!min_ok) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "value below minimum",
            description(),
            std::to_string(value)));
      }
    }

    // Check maximum bound
    if (max_value_.has_value()) {
      bool max_ok = max_exclusive_ ?
        (value < max_value_.value()) :
        (value <= max_value_.value());
      if (!max_ok) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "value above maximum",
            description(),
            std::to_string(value)));
      }
    }

    return ValidationResult::success();
  }

  /// \brief Get a description of this validator's constraints.
  /// \return Human-readable description of the range constraint.
  [[nodiscard]] std::string
  description() const override
  {
    std::ostringstream ss;
    ss << "value in ";

    if (min_value_.has_value() && max_value_.has_value()) {
      ss << (min_exclusive_ ? "(" : "[")
         << min_value_.value()
         << ", "
         << max_value_.value()
         << (max_exclusive_ ? ")" : "]");
    } else if (min_value_.has_value()) {
      ss << (min_exclusive_ ? "(" : "[")
         << min_value_.value()
         << ", +inf)";
    } else if (max_value_.has_value()) {
      ss << "(-inf, "
         << max_value_.value()
         << (max_exclusive_ ? ")" : "]");
    } else {
      ss << "(-inf, +inf)";
    }

    return ss.str();
  }

  /// \brief Get the expected parameter types.
  /// \return Vector containing INTEGER and/or DOUBLE types.
  [[nodiscard]] std::vector<rclcpp::ParameterType>
  expected_types() const override
  {
    if constexpr (std::is_integral_v<T>) {
      return {rclcpp::ParameterType::PARAMETER_INTEGER};
    } else {
      return {
        rclcpp::ParameterType::PARAMETER_INTEGER,
        rclcpp::ParameterType::PARAMETER_DOUBLE
      };
    }
  }

  /// \brief Get the minimum value if set.
  /// \return Optional containing the minimum value, or nullopt.
  [[nodiscard]] std::optional<T>
  get_min() const
  {
    return min_value_;
  }

  /// \brief Get the maximum value if set.
  /// \return Optional containing the maximum value, or nullopt.
  [[nodiscard]] std::optional<T>
  get_max() const
  {
    return max_value_;
  }

  /// \brief Check if minimum bound is exclusive.
  /// \return true if minimum is exclusive, false if inclusive.
  [[nodiscard]] bool
  is_min_exclusive() const
  {
    return min_exclusive_;
  }

  /// \brief Check if maximum bound is exclusive.
  /// \return true if maximum is exclusive, false if inclusive.
  [[nodiscard]] bool
  is_max_exclusive() const
  {
    return max_exclusive_;
  }

private:
  RangeValidator()
  : min_exclusive_(false), max_exclusive_(false)
  {
  }

  std::optional<T> min_value_;
  std::optional<T> max_value_;
  bool min_exclusive_;
  bool max_exclusive_;
};

/// \brief Type alias for integer range validator.
using IntegerRangeValidator = RangeValidator<int64_t>;

/// \brief Type alias for double range validator.
using DoubleRangeValidator = RangeValidator<double>;

/// \brief Validates that a parameter value is one of a set of allowed values.
///
/// This validator checks that the parameter value matches one of the predefined
/// allowed values, similar to an enum constraint.
///
/// Example:
/// \code{.cpp}
/// // String must be one of the allowed modes
/// auto validator = OneOfValidator<std::string>({"slow", "normal", "fast"});
///
/// // Integer must be a valid ID
/// auto validator = OneOfValidator<int64_t>({1, 2, 4, 8, 16});
/// \endcode
///
/// \tparam T The value type (string, int64_t, double, or bool)
template<typename T>
class OneOfValidator : public ParameterValidator
{
public:
  /// \brief Construct from an initializer list of allowed values.
  /// \param allowed_values The set of valid values.
  OneOfValidator(std::initializer_list<T> allowed_values)
  : allowed_values_(allowed_values)
  {
  }

  /// \brief Construct from a container of allowed values.
  /// \tparam Container A container type with begin/end iterators.
  /// \param allowed_values Container of valid values.
  template<typename Container>
  explicit OneOfValidator(const Container & allowed_values)
  : allowed_values_(allowed_values.begin(), allowed_values.end())
  {
  }

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override
  {
    T value;

    // Extract value based on type
    if constexpr (std::is_same_v<T, std::string>) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "string type",
            parameter_type_to_string(param.get_type())));
      }
      value = param.as_string();
    } else if constexpr (std::is_same_v<T, int64_t>) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "integer type",
            parameter_type_to_string(param.get_type())));
      }
      value = param.as_int();
    } else if constexpr (std::is_same_v<T, double>) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "double type",
            parameter_type_to_string(param.get_type())));
      }
      value = param.as_double();
    } else if constexpr (std::is_same_v<T, bool>) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return ValidationResult::failure(
          format_validation_error(
            param.get_name(),
            "type mismatch",
            "bool type",
            parameter_type_to_string(param.get_type())));
      }
      value = param.as_bool();
    }

    // Check if value is in allowed set
    auto it = std::find(allowed_values_.begin(), allowed_values_.end(), value);
    if (it == allowed_values_.end()) {
      return ValidationResult::failure(
        format_validation_error(
          param.get_name(),
          "invalid value",
          description(),
          value_to_string(value)));
    }

    return ValidationResult::success();
  }

  /// \brief Get a description of allowed values.
  /// \return Human-readable description of valid values.
  [[nodiscard]] std::string
  description() const override
  {
    std::ostringstream ss;
    ss << "one of [";
    for (size_t i = 0; i < allowed_values_.size(); ++i) {
      if (i > 0) {
        ss << ", ";
      }
      ss << value_to_string(allowed_values_[i]);
    }
    ss << "]";
    return ss.str();
  }

  /// \brief Get the expected parameter type.
  /// \return Vector with the expected type.
  [[nodiscard]] std::vector<rclcpp::ParameterType>
  expected_types() const override
  {
    if constexpr (std::is_same_v<T, std::string>) {
      return {rclcpp::ParameterType::PARAMETER_STRING};
    } else if constexpr (std::is_same_v<T, int64_t>) {
      return {rclcpp::ParameterType::PARAMETER_INTEGER};
    } else if constexpr (std::is_same_v<T, double>) {
      return {rclcpp::ParameterType::PARAMETER_DOUBLE};
    } else if constexpr (std::is_same_v<T, bool>) {
      return {rclcpp::ParameterType::PARAMETER_BOOL};
    }
    return {};
  }

  /// \brief Get the list of allowed values.
  /// \return Const reference to the allowed values vector.
  [[nodiscard]] const std::vector<T> &
  allowed_values() const
  {
    return allowed_values_;
  }

private:
  template<typename U>
  static std::string value_to_string(const U & val)
  {
    if constexpr (std::is_same_v<U, std::string>) {
      return "\"" + val + "\"";
    } else if constexpr (std::is_same_v<U, bool>) {
      return val ? "true" : "false";
    } else {
      return std::to_string(val);
    }
  }

  std::vector<T> allowed_values_;
};

/// \brief Type alias for string one-of validator.
using StringOneOfValidator = OneOfValidator<std::string>;

/// \brief Type alias for integer one-of validator.
using IntegerOneOfValidator = OneOfValidator<int64_t>;

/// \brief Validates that a string parameter matches a regex pattern.
///
/// This validator uses std::regex to check that string parameter values
/// match the specified pattern.
///
/// Example:
/// \code{.cpp}
/// // Must be a valid identifier
/// auto validator = RegexValidator("^[a-zA-Z_][a-zA-Z0-9_]*$", "valid identifier");
///
/// // Must be an IP address pattern
/// auto validator = RegexValidator(R"(^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$)", "IPv4 address");
/// \endcode
class RCLCPP_PUBLIC RegexValidator : public ParameterValidator
{
public:
  /// \brief Construct a regex validator.
  /// \param pattern The regex pattern string.
  /// \param pattern_description Optional human-readable description of what the pattern matches.
  explicit RegexValidator(
    const std::string & pattern,
    const std::string & pattern_description = "");

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override;

  /// \brief Get a description of the pattern constraint.
  /// \return Human-readable description.
  [[nodiscard]] std::string
  description() const override;

  /// \brief Get the expected parameter type.
  /// \return Vector containing STRING type.
  [[nodiscard]] std::vector<rclcpp::ParameterType>
  expected_types() const override;

  /// \brief Get the regex pattern string.
  /// \return The pattern string.
  [[nodiscard]] const std::string &
  pattern_string() const;

private:
  std::regex pattern_;
  std::string pattern_str_;
  std::string pattern_description_;
};

/// \brief Validates that a string or array parameter is not empty.
///
/// This validator checks that strings have at least one character,
/// and arrays have at least one element.
///
/// Example:
/// \code{.cpp}
/// auto validator = NotEmptyValidator();
/// \endcode
class RCLCPP_PUBLIC NotEmptyValidator : public ParameterValidator
{
public:
  NotEmptyValidator() = default;

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override;

  /// \brief Get a description of the constraint.
  /// \return Human-readable description.
  [[nodiscard]] std::string
  description() const override;
};

/// \brief Validates string length is within specified bounds.
///
/// Example:
/// \code{.cpp}
/// // String must be 1-100 characters
/// auto validator = StringLengthValidator(1, 100);
///
/// // String must be at least 3 characters
/// auto validator = StringLengthValidator::min_length(3);
/// \endcode
class RCLCPP_PUBLIC StringLengthValidator : public ParameterValidator
{
public:
  /// \brief Construct with min and max length bounds.
  /// \param min_len Minimum string length (inclusive).
  /// \param max_len Maximum string length (inclusive).
  StringLengthValidator(size_t min_len, size_t max_len);

  /// \brief Create a validator with only minimum length.
  /// \param min_len Minimum string length.
  /// \return A StringLengthValidator with no upper bound.
  static StringLengthValidator min_length(size_t min_len);

  /// \brief Create a validator with only maximum length.
  /// \param max_len Maximum string length.
  /// \return A StringLengthValidator with no lower bound.
  static StringLengthValidator max_length(size_t max_len);

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override;

  /// \brief Get a description of the length constraint.
  /// \return Human-readable description.
  [[nodiscard]] std::string
  description() const override;

  /// \brief Get the expected parameter type.
  /// \return Vector containing STRING type.
  [[nodiscard]] std::vector<rclcpp::ParameterType>
  expected_types() const override;

private:
  StringLengthValidator();

  std::optional<size_t> min_length_;
  std::optional<size_t> max_length_;
};

/// \brief Validates parameter type matches expected type(s).
///
/// Example:
/// \code{.cpp}
/// // Must be a double
/// auto validator = TypeValidator(rclcpp::ParameterType::PARAMETER_DOUBLE);
///
/// // Must be numeric (int or double)
/// auto validator = TypeValidator({
///   rclcpp::ParameterType::PARAMETER_INTEGER,
///   rclcpp::ParameterType::PARAMETER_DOUBLE
/// });
/// \endcode
class RCLCPP_PUBLIC TypeValidator : public ParameterValidator
{
public:
  /// \brief Construct with a single expected type.
  /// \param expected_type The required parameter type.
  explicit TypeValidator(rclcpp::ParameterType expected_type);

  /// \brief Construct with multiple allowed types.
  /// \param allowed_types List of acceptable parameter types.
  TypeValidator(std::initializer_list<rclcpp::ParameterType> allowed_types);

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult indicating success or failure.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override;

  /// \brief Get a description of the type constraint.
  /// \return Human-readable description.
  [[nodiscard]] std::string
  description() const override;

  /// \brief Get the expected parameter types.
  /// \return Vector of allowed types.
  [[nodiscard]] std::vector<rclcpp::ParameterType>
  expected_types() const override;

private:
  std::vector<rclcpp::ParameterType> allowed_types_;
};

/// \brief Combines multiple validators with AND or OR logic.
///
/// Example:
/// \code{.cpp}
/// // Value must pass ALL validators
/// auto validator = CompositeValidator(CompositeValidator::Mode::ALL)
///   .add(std::make_shared<RangeValidator<double>>(0.0, 100.0))
///   .add(std::make_shared<CustomValidator>());
///
/// // Value must pass ANY validator
/// auto validator = CompositeValidator(CompositeValidator::Mode::ANY)
///   .add(validator1)
///   .add(validator2);
/// \endcode
class RCLCPP_PUBLIC CompositeValidator : public ParameterValidator
{
public:
  /// \brief Composition mode for combining validators.
  enum class Mode
  {
    ALL,  ///< All validators must pass (AND logic)
    ANY   ///< Any validator must pass (OR logic)
  };

  /// \brief Construct a composite validator.
  /// \param mode The composition mode (default: ALL).
  explicit CompositeValidator(Mode mode = Mode::ALL);

  /// \brief Add a validator to the composition.
  /// \param validator The validator to add.
  /// \return Reference to this for chaining.
  CompositeValidator & add(ParameterValidatorPtr validator);

  /// \brief Alias for add() with AND semantics.
  /// \param validator The validator to add.
  /// \return Reference to this for chaining.
  CompositeValidator & and_also(ParameterValidatorPtr validator);

  /// \brief Add a validator and switch to ANY mode.
  /// \param validator The validator to add.
  /// \return Reference to this for chaining.
  CompositeValidator & or_else(ParameterValidatorPtr validator);

  /// \brief Validate the parameter value.
  /// \param param The parameter to validate.
  /// \return ValidationResult based on composition mode.
  [[nodiscard]] ValidationResult
  validate(const rclcpp::Parameter & param) const override;

  /// \brief Get a description of all composed constraints.
  /// \return Human-readable description.
  [[nodiscard]] std::string
  description() const override;

  /// \brief Get the composition mode.
  /// \return The current Mode.
  [[nodiscard]] Mode
  mode() const;

  /// \brief Get the number of validators.
  /// \return Count of composed validators.
  [[nodiscard]] size_t
  size() const;

private:
  Mode mode_;
  std::vector<ParameterValidatorPtr> validators_;
};

/// \brief Convenience factory functions for creating validators.
namespace validators
{

/// \brief Create a range validator with both bounds.
/// \tparam T The numeric type.
/// \param min_val Minimum value (inclusive).
/// \param max_val Maximum value (inclusive).
/// \return Shared pointer to the validator.
template<typename T>
ParameterValidatorPtr range(T min_val, T max_val)
{
  return std::make_shared<RangeValidator<T>>(min_val, max_val);
}

/// \brief Create a range validator with only minimum bound.
/// \tparam T The numeric type.
/// \param min_val Minimum value (inclusive).
/// \return Shared pointer to the validator.
template<typename T>
ParameterValidatorPtr min(T min_val)
{
  return std::make_shared<RangeValidator<T>>(RangeValidator<T>::min(min_val));
}

/// \brief Create a range validator with only maximum bound.
/// \tparam T The numeric type.
/// \param max_val Maximum value (inclusive).
/// \return Shared pointer to the validator.
template<typename T>
ParameterValidatorPtr max(T max_val)
{
  return std::make_shared<RangeValidator<T>>(RangeValidator<T>::max(max_val));
}

/// \brief Create a one-of validator from allowed values.
/// \tparam T The value type.
/// \param values Initializer list of allowed values.
/// \return Shared pointer to the validator.
template<typename T>
ParameterValidatorPtr one_of(std::initializer_list<T> values)
{
  return std::make_shared<OneOfValidator<T>>(values);
}

/// \brief Create a regex validator.
/// \param pattern The regex pattern.
/// \param description Optional description of the pattern.
/// \return Shared pointer to the validator.
RCLCPP_PUBLIC
ParameterValidatorPtr matches(
  const std::string & pattern,
  const std::string & description = "");

/// \brief Create a not-empty validator.
/// \return Shared pointer to the validator.
RCLCPP_PUBLIC
ParameterValidatorPtr not_empty();

/// \brief Create a string length validator.
/// \param min_len Minimum length.
/// \param max_len Maximum length.
/// \return Shared pointer to the validator.
RCLCPP_PUBLIC
ParameterValidatorPtr length(size_t min_len, size_t max_len);

/// \brief Create a type validator.
/// \param expected The expected parameter type.
/// \return Shared pointer to the validator.
RCLCPP_PUBLIC
ParameterValidatorPtr type(rclcpp::ParameterType expected);

/// \brief Create a composite validator requiring all validators to pass.
/// \param validators List of validators to compose.
/// \return Shared pointer to the composite validator.
RCLCPP_PUBLIC
ParameterValidatorPtr all_of(std::initializer_list<ParameterValidatorPtr> validators);

/// \brief Create a composite validator requiring any validator to pass.
/// \param validators List of validators to compose.
/// \return Shared pointer to the composite validator.
RCLCPP_PUBLIC
ParameterValidatorPtr any_of(std::initializer_list<ParameterValidatorPtr> validators);

}  // namespace validators

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_VALIDATORS_HPP_
