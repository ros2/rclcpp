// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::ParameterType;
using rclcpp::parameter::ParameterVariant;

ParameterVariant::ParameterVariant()
: name_("")
{
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
}

ParameterVariant::ParameterVariant(const std::string & name, const bool bool_value)
: name_(name)
{
  value_.bool_value = bool_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
}

ParameterVariant::ParameterVariant(const std::string & name, const int int_value)
: name_(name)
{
  value_.integer_value = int_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

ParameterVariant::ParameterVariant(const std::string & name, const int64_t int_value)
: name_(name)
{
  value_.integer_value = int_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

ParameterVariant::ParameterVariant(const std::string & name, const float double_value)
: name_(name)
{
  value_.double_value = double_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

ParameterVariant::ParameterVariant(const std::string & name, const double double_value)
: name_(name)
{
  value_.double_value = double_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

ParameterVariant::ParameterVariant(const std::string & name, const std::string & string_value)
: name_(name)
{
  value_.string_value = string_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
}

ParameterVariant::ParameterVariant(const std::string & name, const char * string_value)
: ParameterVariant(name, std::string(string_value))
{}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<uint8_t> & byte_array_value)
: name_(name)
{
  value_.byte_array_value = byte_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<bool> & bool_array_value)
: name_(name)
{
  value_.bool_array_value = bool_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<int> & int_array_value)
: name_(name)
{
  vector_assign(value_.integer_array_value, int_array_value);
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<int64_t> & int_array_value)
: name_(name)
{
  value_.integer_array_value = int_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<float> & double_array_value)
: name_(name)
{
  vector_assign(value_.double_array_value, double_array_value);
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<double> & double_array_value)
: name_(name)
{
  value_.double_array_value = double_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

ParameterVariant::ParameterVariant(
  const std::string & name, const std::vector<std::string> & string_array_value)
: name_(name)
{
  value_.string_array_value = string_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
}

ParameterType
ParameterVariant::get_type() const
{
  return static_cast<ParameterType>(value_.type);
}

std::string
ParameterVariant::get_type_name() const
{
  return rclcpp::to_string(get_type());
}

const std::string &
ParameterVariant::get_name() const
{
  return name_;
}

rcl_interfaces::msg::ParameterValue
ParameterVariant::get_parameter_value() const
{
  return value_;
}

bool
ParameterVariant::as_bool() const
{
  return get_value<ParameterType::PARAMETER_BOOL>();
}

int64_t
ParameterVariant::as_int() const
{
  return get_value<ParameterType::PARAMETER_INTEGER>();
}

double
ParameterVariant::as_double() const
{
  return get_value<ParameterType::PARAMETER_DOUBLE>();
}

const std::string &
ParameterVariant::as_string() const
{
  return get_value<ParameterType::PARAMETER_STRING>();
}

const std::vector<uint8_t> &
ParameterVariant::as_byte_array() const
{
  return get_value<ParameterType::PARAMETER_BYTE_ARRAY>();
}

const std::vector<bool> &
ParameterVariant::as_bool_array() const
{
  return get_value<ParameterType::PARAMETER_BOOL_ARRAY>();
}

const std::vector<int64_t> &
ParameterVariant::as_integer_array() const
{
  return get_value<ParameterType::PARAMETER_INTEGER_ARRAY>();
}

const std::vector<double> &
ParameterVariant::as_double_array() const
{
  return get_value<ParameterType::PARAMETER_DOUBLE_ARRAY>();
}

const std::vector<std::string> &
ParameterVariant::as_string_array() const
{
  return get_value<ParameterType::PARAMETER_STRING_ARRAY>();
}

ParameterVariant
ParameterVariant::from_parameter(const rcl_interfaces::msg::Parameter & parameter)
{
  switch (parameter.value.type) {
    case PARAMETER_NOT_SET:
      throw std::runtime_error("Type from ParameterValue is not set");
    case PARAMETER_BOOL:
      return ParameterVariant(parameter.name, parameter.value.bool_value);
    case PARAMETER_INTEGER:
      return ParameterVariant(parameter.name, parameter.value.integer_value);
    case PARAMETER_DOUBLE:
      return ParameterVariant(parameter.name, parameter.value.double_value);
    case PARAMETER_STRING:
      return ParameterVariant(parameter.name, parameter.value.string_value);
    case PARAMETER_BYTE_ARRAY:
      return ParameterVariant(parameter.name, parameter.value.byte_array_value);
    case PARAMETER_BOOL_ARRAY:
      return ParameterVariant(parameter.name, parameter.value.bool_array_value);
    case PARAMETER_INTEGER_ARRAY:
      return ParameterVariant(parameter.name, parameter.value.integer_array_value);
    case PARAMETER_DOUBLE_ARRAY:
      return ParameterVariant(parameter.name, parameter.value.double_array_value);
    case PARAMETER_STRING_ARRAY:
      return ParameterVariant(parameter.name, parameter.value.string_array_value);
    default:
      // TODO(wjwwood): use custom exception
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        "Unexpected type from ParameterVariant: " + std::to_string(parameter.value.type));
      // *INDENT-ON*
  }
}

rcl_interfaces::msg::Parameter
ParameterVariant::to_parameter()
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name_;
  parameter.value = value_;
  return parameter;
}

std::string
ParameterVariant::value_to_string() const
{
  switch (get_type()) {
    case ParameterType::PARAMETER_NOT_SET:
      return "not set";
    case ParameterType::PARAMETER_BOOL:
      return as_bool() ? "true" : "false";
    case ParameterType::PARAMETER_INTEGER:
      return std::to_string(as_int());
    case ParameterType::PARAMETER_DOUBLE:
      return std::to_string(as_double());
    case ParameterType::PARAMETER_STRING:
      return as_string();
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return array_to_string<uint8_t, int>(as_byte_array(), std::ios::hex);
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return array_to_string(as_bool_array(), std::ios::boolalpha);
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return array_to_string(as_integer_array());
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return array_to_string(as_double_array());
    case ParameterType::PARAMETER_STRING_ARRAY:
      return array_to_string(as_string_array());
    default:
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
        "Unexpected type from ParameterVariant: " + std::to_string(get_type()));
      // *INDENT-ON*
  }
}

std::string
rclcpp::parameter::_to_json_dict_entry(const ParameterVariant &param)
{
  std::stringstream ss;
  ss << "\"" << param.get_name() << "\": ";
  ss << "{\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

std::ostream &
rclcpp::parameter::operator<<(std::ostream & os, const rclcpp::parameter::ParameterVariant & pv)
{
  os << std::to_string(pv);
  return os;
}

std::ostream &
rclcpp::parameter::operator<<(std::ostream & os, const std::vector<ParameterVariant> & parameters)
{
  os << std::to_string(parameters);
  return os;
}

std::string
std::to_string(const rclcpp::parameter::ParameterVariant & param)
{
  std::stringstream ss;
  ss << "{\"name\": \"" << param.get_name() << "\", ";
  ss << "\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

std::string
std::to_string(const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::stringstream ss;
  ss << "{";
  bool first = true;
  for (const auto & pv : parameters) {
    if (first == false) {
      ss << ", ";
    } else {
      first = false;
    }
    ss << rclcpp::parameter::_to_json_dict_entry(pv);
  }
  ss << "}";
  return ss.str();
}
