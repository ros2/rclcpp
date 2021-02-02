// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/parameter_value.hpp"

#include <string>
#include <vector>

using rclcpp::ParameterType;
using rclcpp::ParameterValue;

std::string
rclcpp::to_string(const ParameterType type)
{
  switch (type) {
    case ParameterType::PARAMETER_NOT_SET:
      return "not set";
    case ParameterType::PARAMETER_BOOL:
      return "bool";
    case ParameterType::PARAMETER_INTEGER:
      return "integer";
    case ParameterType::PARAMETER_DOUBLE:
      return "double";
    case ParameterType::PARAMETER_STRING:
      return "string";
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return "integer_array";
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    default:
      return "unknown type";
  }
}

std::ostream &
rclcpp::operator<<(std::ostream & os, const ParameterType type)
{
  os << rclcpp::to_string(type);
  return os;
}

template<typename ValType, typename PrintType = ValType>
std::string
array_to_string(
  const std::vector<ValType> & array,
  const std::ios::fmtflags format_flags = std::ios::dec)
{
  std::stringstream type_array;
  bool first_item = true;
  type_array << "[";
  type_array.setf(format_flags, std::ios_base::basefield | std::ios::boolalpha);
  type_array << std::showbase;
  for (const ValType & value : array) {
    if (!first_item) {
      type_array << ", ";
    } else {
      first_item = false;
    }
    type_array << static_cast<PrintType>(value);
  }
  type_array << "]";
  return type_array.str();
}

std::string
rclcpp::to_string(const ParameterValue & value)
{
  switch (value.get_type()) {
    case ParameterType::PARAMETER_NOT_SET:
      return "not set";
    case ParameterType::PARAMETER_BOOL:
      return value.get<bool>() ? "true" : "false";
    case ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.get<int>());
    case ParameterType::PARAMETER_DOUBLE:
      return std::to_string(value.get<double>());
    case ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return array_to_string<uint8_t, int>(value.get<std::vector<uint8_t>>(), std::ios::hex);
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return array_to_string(value.get<std::vector<bool>>(), std::ios::boolalpha);
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return array_to_string(value.get<std::vector<int64_t>>());
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return array_to_string(value.get<std::vector<double>>());
    case ParameterType::PARAMETER_STRING_ARRAY:
      return array_to_string(value.get<std::vector<std::string>>());
    default:
      return "unknown type";
  }
}

ParameterValue::ParameterValue()
{
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
}

ParameterValue::ParameterValue(const rcl_interfaces::msg::ParameterValue & value)
{
  value_ = value;
  switch (value.type) {
    case PARAMETER_BOOL:
    case PARAMETER_INTEGER:
    case PARAMETER_DOUBLE:
    case PARAMETER_STRING:
    case PARAMETER_BYTE_ARRAY:
    case PARAMETER_BOOL_ARRAY:
    case PARAMETER_INTEGER_ARRAY:
    case PARAMETER_DOUBLE_ARRAY:
    case PARAMETER_STRING_ARRAY:
    case PARAMETER_NOT_SET:
      break;
    default:
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Unknown type: " + std::to_string(value.type));
  }
}

ParameterValue::ParameterValue(const bool bool_value)
{
  value_.bool_value = bool_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
}

ParameterValue::ParameterValue(const int int_value)
{
  value_.integer_value = int_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

ParameterValue::ParameterValue(const int64_t int_value)
{
  value_.integer_value = int_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

ParameterValue::ParameterValue(const float double_value)
{
  value_.double_value = static_cast<double>(double_value);
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

ParameterValue::ParameterValue(const double double_value)
{
  value_.double_value = double_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

ParameterValue::ParameterValue(const std::string & string_value)
{
  value_.string_value = string_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
}

ParameterValue::ParameterValue(const char * string_value)
: ParameterValue(std::string(string_value))
{}

ParameterValue::ParameterValue(const std::vector<uint8_t> & byte_array_value)
{
  value_.byte_array_value = byte_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<bool> & bool_array_value)
{
  value_.bool_array_value = bool_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<int> & int_array_value)
{
  value_.integer_array_value.assign(int_array_value.cbegin(), int_array_value.cend());
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<int64_t> & int_array_value)
{
  value_.integer_array_value = int_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<float> & float_array_value)
{
  value_.double_array_value.assign(float_array_value.cbegin(), float_array_value.cend());
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<double> & double_array_value)
{
  value_.double_array_value = double_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

ParameterValue::ParameterValue(const std::vector<std::string> & string_array_value)
{
  value_.string_array_value = string_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
}

ParameterType
ParameterValue::get_type() const
{
  return static_cast<ParameterType>(value_.type);
}

rcl_interfaces::msg::ParameterValue
ParameterValue::to_value_msg() const
{
  return value_;
}

bool
ParameterValue::operator==(const ParameterValue & rhs) const
{
  return this->value_ == rhs.value_;
}

bool
ParameterValue::operator!=(const ParameterValue & rhs) const
{
  return this->value_ != rhs.value_;
}
