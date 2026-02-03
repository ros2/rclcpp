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

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/parameter_validation.hpp"
#include "rclcpp/parameter_validators.hpp"

namespace rclcpp
{

// =============================================================================
// ValidationResult Implementation
// =============================================================================

ValidationResult ValidationResult::success()
{
  return ValidationResult(true, "");
}

ValidationResult ValidationResult::failure(const std::string & reason)
{
  return ValidationResult(false, reason);
}

ValidationResult::ValidationResult(bool successful, std::string reason)
: successful_(successful), reason_(std::move(reason))
{
}

bool ValidationResult::successful() const noexcept
{
  return successful_;
}

ValidationResult::operator bool() const noexcept
{
  return successful_;
}

const std::string & ValidationResult::reason() const noexcept
{
  return reason_;
}

rcl_interfaces::msg::SetParametersResult ValidationResult::to_msg() const
{
  rcl_interfaces::msg::SetParametersResult msg;
  msg.successful = successful_;
  msg.reason = reason_;
  return msg;
}

// =============================================================================
// ParameterValidator Implementation
// =============================================================================

std::vector<rclcpp::ParameterType> ParameterValidator::expected_types() const
{
  return {};  // Empty means any type is accepted
}

// =============================================================================
// Utility Functions
// =============================================================================

std::string format_validation_error(
  const std::string & param_name,
  const std::string & reason,
  const std::string & expected,
  const std::string & actual)
{
  std::ostringstream ss;
  ss << "Parameter '" << param_name << "' validation failed: " << reason << "\n"
     << "  Expected: " << expected << "\n"
     << "  Got: " << actual;
  return ss.str();
}

std::string parameter_value_to_string(const rclcpp::Parameter & param)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return "<not set>";
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param.as_bool() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(param.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return std::to_string(param.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "\"" + param.as_string() + "\"";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      {
        auto arr = param.as_byte_array();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < std::min(arr.size(), size_t(5)); ++i) {
          if (i > 0) {ss << ", ";}
          ss << static_cast<int>(arr[i]);
        }
        if (arr.size() > 5) {ss << ", ...";}
        ss << "] (size=" << arr.size() << ")";
        return ss.str();
      }
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      {
        auto arr = param.as_bool_array();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < std::min(arr.size(), size_t(5)); ++i) {
          if (i > 0) {ss << ", ";}
          ss << (arr[i] ? "true" : "false");
        }
        if (arr.size() > 5) {ss << ", ...";}
        ss << "] (size=" << arr.size() << ")";
        return ss.str();
      }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      {
        auto arr = param.as_integer_array();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < std::min(arr.size(), size_t(5)); ++i) {
          if (i > 0) {ss << ", ";}
          ss << arr[i];
        }
        if (arr.size() > 5) {ss << ", ...";}
        ss << "] (size=" << arr.size() << ")";
        return ss.str();
      }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      {
        auto arr = param.as_double_array();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < std::min(arr.size(), size_t(5)); ++i) {
          if (i > 0) {ss << ", ";}
          ss << arr[i];
        }
        if (arr.size() > 5) {ss << ", ...";}
        ss << "] (size=" << arr.size() << ")";
        return ss.str();
      }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      {
        auto arr = param.as_string_array();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < std::min(arr.size(), size_t(5)); ++i) {
          if (i > 0) {ss << ", ";}
          ss << "\"" << arr[i] << "\"";
        }
        if (arr.size() > 5) {ss << ", ...";}
        ss << "] (size=" << arr.size() << ")";
        return ss.str();
      }
    default:
      return "<unknown>";
  }
}

std::string parameter_type_to_string(rclcpp::ParameterType type)
{
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return "not_set";
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return "integer";
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return "double";
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "string";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return "integer_array";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    default:
      return "unknown";
  }
}

// =============================================================================
// RegexValidator Implementation
// =============================================================================

RegexValidator::RegexValidator(
  const std::string & pattern,
  const std::string & pattern_description)
: pattern_(pattern, std::regex::ECMAScript),
  pattern_str_(pattern),
  pattern_description_(pattern_description)
{
}

ValidationResult RegexValidator::validate(const rclcpp::Parameter & param) const
{
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "type mismatch",
        "string type",
        parameter_type_to_string(param.get_type())));
  }

  const std::string & value = param.as_string();
  if (!std::regex_match(value, pattern_)) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "pattern mismatch",
        description(),
        "\"" + value + "\""));
  }

  return ValidationResult::success();
}

std::string RegexValidator::description() const
{
  std::ostringstream ss;
  ss << "string matching pattern \"" << pattern_str_ << "\"";
  if (!pattern_description_.empty()) {
    ss << " (" << pattern_description_ << ")";
  }
  return ss.str();
}

std::vector<rclcpp::ParameterType> RegexValidator::expected_types() const
{
  return {rclcpp::ParameterType::PARAMETER_STRING};
}

const std::string & RegexValidator::pattern_string() const
{
  return pattern_str_;
}

// =============================================================================
// NotEmptyValidator Implementation
// =============================================================================

ValidationResult NotEmptyValidator::validate(const rclcpp::Parameter & param) const
{
  bool is_empty = false;
  std::string type_name;

  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING:
      is_empty = param.as_string().empty();
      type_name = "string";
      break;
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      is_empty = param.as_byte_array().empty();
      type_name = "byte array";
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      is_empty = param.as_bool_array().empty();
      type_name = "bool array";
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      is_empty = param.as_integer_array().empty();
      type_name = "integer array";
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      is_empty = param.as_double_array().empty();
      type_name = "double array";
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      is_empty = param.as_string_array().empty();
      type_name = "string array";
      break;
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      is_empty = true;
      type_name = "parameter";
      break;
    default:
      // For scalar types (bool, int, double), they are never "empty"
      return ValidationResult::success();
  }

  if (is_empty) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "empty " + type_name,
        "non-empty " + type_name,
        "empty"));
  }

  return ValidationResult::success();
}

std::string NotEmptyValidator::description() const
{
  return "non-empty value";
}

// =============================================================================
// StringLengthValidator Implementation
// =============================================================================

StringLengthValidator::StringLengthValidator(size_t min_len, size_t max_len)
: min_length_(min_len), max_length_(max_len)
{
}

StringLengthValidator::StringLengthValidator()
{
}

StringLengthValidator StringLengthValidator::min_length(size_t min_len)
{
  StringLengthValidator v;
  v.min_length_ = min_len;
  return v;
}

StringLengthValidator StringLengthValidator::max_length(size_t max_len)
{
  StringLengthValidator v;
  v.max_length_ = max_len;
  return v;
}

ValidationResult StringLengthValidator::validate(const rclcpp::Parameter & param) const
{
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "type mismatch",
        "string type",
        parameter_type_to_string(param.get_type())));
  }

  size_t len = param.as_string().length();

  if (min_length_.has_value() && len < min_length_.value()) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "string too short",
        description(),
        "length " + std::to_string(len)));
  }

  if (max_length_.has_value() && len > max_length_.value()) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "string too long",
        description(),
        "length " + std::to_string(len)));
  }

  return ValidationResult::success();
}

std::string StringLengthValidator::description() const
{
  std::ostringstream ss;
  ss << "string with length ";

  if (min_length_.has_value() && max_length_.has_value()) {
    ss << "in [" << min_length_.value() << ", " << max_length_.value() << "]";
  } else if (min_length_.has_value()) {
    ss << ">= " << min_length_.value();
  } else if (max_length_.has_value()) {
    ss << "<= " << max_length_.value();
  } else {
    ss << "any";
  }

  return ss.str();
}

std::vector<rclcpp::ParameterType> StringLengthValidator::expected_types() const
{
  return {rclcpp::ParameterType::PARAMETER_STRING};
}

// =============================================================================
// TypeValidator Implementation
// =============================================================================

TypeValidator::TypeValidator(rclcpp::ParameterType expected_type)
: allowed_types_({expected_type})
{
}

TypeValidator::TypeValidator(std::initializer_list<rclcpp::ParameterType> allowed_types)
: allowed_types_(allowed_types)
{
}

ValidationResult TypeValidator::validate(const rclcpp::Parameter & param) const
{
  auto it = std::find(allowed_types_.begin(), allowed_types_.end(), param.get_type());
  if (it == allowed_types_.end()) {
    return ValidationResult::failure(
      format_validation_error(
        param.get_name(),
        "type mismatch",
        description(),
        parameter_type_to_string(param.get_type())));
  }

  return ValidationResult::success();
}

std::string TypeValidator::description() const
{
  if (allowed_types_.size() == 1) {
    return "type " + parameter_type_to_string(allowed_types_[0]);
  }

  std::ostringstream ss;
  ss << "one of types [";
  for (size_t i = 0; i < allowed_types_.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << parameter_type_to_string(allowed_types_[i]);
  }
  ss << "]";
  return ss.str();
}

std::vector<rclcpp::ParameterType> TypeValidator::expected_types() const
{
  return allowed_types_;
}

// =============================================================================
// CompositeValidator Implementation
// =============================================================================

CompositeValidator::CompositeValidator(Mode mode)
: mode_(mode)
{
}

CompositeValidator & CompositeValidator::add(ParameterValidatorPtr validator)
{
  validators_.push_back(std::move(validator));
  return *this;
}

CompositeValidator & CompositeValidator::and_also(ParameterValidatorPtr validator)
{
  return add(std::move(validator));
}

CompositeValidator & CompositeValidator::or_else(ParameterValidatorPtr validator)
{
  mode_ = Mode::ANY;
  return add(std::move(validator));
}

ValidationResult CompositeValidator::validate(const rclcpp::Parameter & param) const
{
  if (validators_.empty()) {
    return ValidationResult::success();
  }

  std::vector<std::string> failure_reasons;

  for (const auto & validator : validators_) {
    auto result = validator->validate(param);

    if (mode_ == Mode::ALL) {
      // ALL mode: fail on first failure
      if (!result) {
        return result;
      }
    } else {
      // ANY mode: succeed on first success
      if (result) {
        return ValidationResult::success();
      }
      failure_reasons.push_back(result.reason());
    }
  }

  if (mode_ == Mode::ALL) {
    return ValidationResult::success();
  } else {
    // ALL validators failed in ANY mode
    std::ostringstream ss;
    ss << "Parameter '" << param.get_name() << "' validation failed: "
       << "none of the validators passed\n";
    for (size_t i = 0; i < failure_reasons.size(); ++i) {
      ss << "  Validator " << (i + 1) << ": " << failure_reasons[i];
      if (i < failure_reasons.size() - 1) {
        ss << "\n";
      }
    }
    return ValidationResult::failure(ss.str());
  }
}

std::string CompositeValidator::description() const
{
  if (validators_.empty()) {
    return "any value";
  }

  std::ostringstream ss;
  std::string connector = (mode_ == Mode::ALL) ? " AND " : " OR ";

  ss << "(";
  for (size_t i = 0; i < validators_.size(); ++i) {
    if (i > 0) {
      ss << connector;
    }
    ss << validators_[i]->description();
  }
  ss << ")";

  return ss.str();
}

CompositeValidator::Mode CompositeValidator::mode() const
{
  return mode_;
}

size_t CompositeValidator::size() const
{
  return validators_.size();
}

// =============================================================================
// Factory Functions
// =============================================================================

namespace validators
{

ParameterValidatorPtr matches(
  const std::string & pattern,
  const std::string & description)
{
  return std::make_shared<RegexValidator>(pattern, description);
}

ParameterValidatorPtr not_empty()
{
  return std::make_shared<NotEmptyValidator>();
}

ParameterValidatorPtr length(size_t min_len, size_t max_len)
{
  return std::make_shared<StringLengthValidator>(min_len, max_len);
}

ParameterValidatorPtr type(rclcpp::ParameterType expected)
{
  return std::make_shared<TypeValidator>(expected);
}

ParameterValidatorPtr all_of(std::initializer_list<ParameterValidatorPtr> validators)
{
  auto composite = std::make_shared<CompositeValidator>(CompositeValidator::Mode::ALL);
  for (const auto & v : validators) {
    composite->add(v);
  }
  return composite;
}

ParameterValidatorPtr any_of(std::initializer_list<ParameterValidatorPtr> validators)
{
  auto composite = std::make_shared<CompositeValidator>(CompositeValidator::Mode::ANY);
  for (const auto & v : validators) {
    composite->add(v);
  }
  return composite;
}

}  // namespace validators

}  // namespace rclcpp
