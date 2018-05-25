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
using rclcpp::ParameterTypeException;
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

ParameterTypeException::ParameterTypeException(ParameterType expected, ParameterType actual)
: msg_("expected [" + rclcpp::to_string(expected) + "] got [" + rclcpp::to_string(actual) + "]")
{
}

ParameterTypeException::~ParameterTypeException()
{
}

const char *
ParameterTypeException::what() const throw ()
{
  return msg_.c_str();
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
  for (const ValType value : array) {
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
to_string(const ParameterValue & value)
{
  switch (value.get_type()) {
    case ParameterType::PARAMETER_NOT_SET:
      return "not set";
    case ParameterType::PARAMETER_BOOL:
      return value.as_bool() ? "true" : "false";
    case ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.as_int());
    case ParameterType::PARAMETER_DOUBLE:
      return std::to_string(value.as_double());
    case ParameterType::PARAMETER_STRING:
      return value.as_string();
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return array_to_string<uint8_t, int>(value.as_byte_array(), std::ios::hex);
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return array_to_string(value.as_bool_array(), std::ios::boolalpha);
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return array_to_string(value.as_integer_array());
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return array_to_string(value.as_double_array());
    case ParameterType::PARAMETER_STRING_ARRAY:
      return array_to_string(value.as_string_array());
    default:
      return "unknown type";
  }
}

ParameterValue::ParameterValue()
{
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
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
  value_.double_value = double_value;
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

ParameterValue::ParameterValue(const std::vector<float> & double_array_value)
{
  value_.integer_array_value.assign(double_array_value.cbegin(), double_array_value.cend());
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
ParameterValue::get_value_message() const
{
  return value_;
}

bool
ParameterValue::as_bool() const
{
  return get<ParameterType::PARAMETER_BOOL>();
}

int64_t
ParameterValue::as_int() const
{
  return get<ParameterType::PARAMETER_INTEGER>();
}

double
ParameterValue::as_double() const
{
  return get<ParameterType::PARAMETER_DOUBLE>();
}

const std::string &
ParameterValue::as_string() const
{
  return get<ParameterType::PARAMETER_STRING>();
}

const std::vector<uint8_t> &
ParameterValue::as_byte_array() const
{
  return get<ParameterType::PARAMETER_BYTE_ARRAY>();
}

const std::vector<bool> &
ParameterValue::as_bool_array() const
{
  return get<ParameterType::PARAMETER_BOOL_ARRAY>();
}

const std::vector<int64_t> &
ParameterValue::as_integer_array() const
{
  return get<ParameterType::PARAMETER_INTEGER_ARRAY>();
}

const std::vector<double> &
ParameterValue::as_double_array() const
{
  return get<ParameterType::PARAMETER_DOUBLE_ARRAY>();
}

const std::vector<std::string> &
ParameterValue::as_string_array() const
{
  return get<ParameterType::PARAMETER_STRING_ARRAY>();
}
