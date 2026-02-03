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

#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/parameter_builder.hpp"

namespace rclcpp
{

// =============================================================================
// ParameterBuilder Implementation
// =============================================================================

ParameterBuilder::ParameterBuilder(const std::string & param_name)
: name_(param_name)
{
}

ParameterBuilder & ParameterBuilder::name(const std::string & param_name)
{
  name_ = param_name;
  return *this;
}

ParameterBuilder & ParameterBuilder::description(const std::string & desc)
{
  description_ = desc;
  return *this;
}

ParameterBuilder & ParameterBuilder::read_only(bool is_read_only)
{
  read_only_ = is_read_only;
  return *this;
}

ParameterBuilder & ParameterBuilder::default_value(const rclcpp::ParameterValue & value)
{
  default_value_ = value;
  return *this;
}

ParameterBuilder & ParameterBuilder::validator(ParameterValidatorPtr v)
{
  validators_.push_back(std::move(v));
  return *this;
}

ParameterBuilder & ParameterBuilder::matches(
  const std::string & pattern,
  const std::string & pattern_desc)
{
  validators_.push_back(std::make_shared<RegexValidator>(pattern, pattern_desc));
  return *this;
}

ParameterBuilder & ParameterBuilder::not_empty()
{
  validators_.push_back(std::make_shared<NotEmptyValidator>());
  return *this;
}

ParameterBuilder & ParameterBuilder::length(size_t min_len, size_t max_len)
{
  validators_.push_back(std::make_shared<StringLengthValidator>(min_len, max_len));
  return *this;
}

const std::string & ParameterBuilder::get_name() const
{
  return name_;
}

const std::string & ParameterBuilder::get_description() const
{
  return description_;
}

bool ParameterBuilder::is_read_only() const
{
  return read_only_;
}

bool ParameterBuilder::has_default_value() const
{
  return default_value_.has_value();
}

const rclcpp::ParameterValue & ParameterBuilder::get_default_value() const
{
  if (!default_value_.has_value()) {
    throw std::runtime_error("No default value set for parameter '" + name_ + "'");
  }
  return default_value_.value();
}

rclcpp::Parameter ParameterBuilder::build() const
{
  if (name_.empty()) {
    throw std::runtime_error("Parameter name must be set before building");
  }

  if (default_value_.has_value()) {
    return rclcpp::Parameter(name_, default_value_.value());
  }

  return rclcpp::Parameter(name_);
}

rcl_interfaces::msg::ParameterDescriptor ParameterBuilder::build_descriptor() const
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description_;
  descriptor.read_only = read_only_;

  // Build additional_constraints from validator descriptions
  if (!validators_.empty()) {
    std::ostringstream ss;
    ss << "Constraints: ";
    for (size_t i = 0; i < validators_.size(); ++i) {
      if (i > 0) {
        ss << "; ";
      }
      ss << validators_[i]->description();
    }
    descriptor.additional_constraints = ss.str();
  }

  // Populate range info from RangeValidator if present
  for (const auto & validator : validators_) {
    // Check for double range validator
    auto double_range = std::dynamic_pointer_cast<RangeValidator<double>>(validator);
    if (double_range) {
      auto min_val = double_range->get_min();
      auto max_val = double_range->get_max();

      if (min_val.has_value() || max_val.has_value()) {
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min_val.value_or(-std::numeric_limits<double>::infinity());
        range.to_value = max_val.value_or(std::numeric_limits<double>::infinity());
        range.step = 0.0;  // Any step
        descriptor.floating_point_range.push_back(range);
      }
      break;  // Only use first range validator found
    }

    // Check for integer range validator
    auto int_range = std::dynamic_pointer_cast<RangeValidator<int64_t>>(validator);
    if (int_range) {
      auto min_val = int_range->get_min();
      auto max_val = int_range->get_max();

      if (min_val.has_value() || max_val.has_value()) {
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = min_val.value_or(std::numeric_limits<int64_t>::min());
        range.to_value = max_val.value_or(std::numeric_limits<int64_t>::max());
        range.step = 0;  // Any step
        descriptor.integer_range.push_back(range);
      }
      break;
    }
  }

  return descriptor;
}

const std::vector<ParameterValidatorPtr> & ParameterBuilder::get_validators() const
{
  return validators_;
}

ValidationResult ParameterBuilder::validate(const rclcpp::Parameter & param) const
{
  for (const auto & validator : validators_) {
    auto result = validator->validate(param);
    if (!result) {
      return result;
    }
  }
  return ValidationResult::success();
}

// =============================================================================
// ParameterValidatorRegistry Implementation
// =============================================================================

void ParameterValidatorRegistry::register_validators(
  const std::string & param_name,
  std::vector<ParameterValidatorPtr> validators)
{
  std::lock_guard<std::mutex> lock(mutex_);
  validators_[param_name] = std::move(validators);
}

void ParameterValidatorRegistry::add_validator(
  const std::string & param_name,
  ParameterValidatorPtr validator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  validators_[param_name].push_back(std::move(validator));
}

void ParameterValidatorRegistry::clear_validators(const std::string & param_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  validators_.erase(param_name);
}

bool ParameterValidatorRegistry::has_validators(const std::string & param_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = validators_.find(param_name);
  return it != validators_.end() && !it->second.empty();
}

ValidationResult ParameterValidatorRegistry::validate(const rclcpp::Parameter & param) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = validators_.find(param.get_name());
  if (it == validators_.end()) {
    return ValidationResult::success();
  }

  for (const auto & validator : it->second) {
    auto result = validator->validate(param);
    if (!result) {
      return result;
    }
  }

  return ValidationResult::success();
}

ValidationResult ParameterValidatorRegistry::validate(
  const std::vector<rclcpp::Parameter> & params) const
{
  for (const auto & param : params) {
    auto result = validate(param);
    if (!result) {
      return result;
    }
  }
  return ValidationResult::success();
}

std::function<rcl_interfaces::msg::SetParametersResult(
    const std::vector<rclcpp::Parameter> &)>
ParameterValidatorRegistry::create_callback() const
{
  // Capture a copy of the validators for thread safety
  std::map<std::string, std::vector<ParameterValidatorPtr>> validators_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    validators_copy = validators_;
  }

  return [validators_copy](
    const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto & param : params) {
        auto it = validators_copy.find(param.get_name());
        if (it != validators_copy.end()) {
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
}

// =============================================================================
// Free Functions
// =============================================================================

ValidationResult validate_parameter(
  const rclcpp::Parameter & param,
  const std::vector<ParameterValidatorPtr> & validators)
{
  for (const auto & validator : validators) {
    auto result = validator->validate(param);
    if (!result) {
      return result;
    }
  }
  return ValidationResult::success();
}

}  // namespace rclcpp
