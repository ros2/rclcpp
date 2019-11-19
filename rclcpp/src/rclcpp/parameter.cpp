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

#include "rclcpp/parameter.hpp"

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::ParameterType;
using rclcpp::Parameter;

Parameter::Parameter()
: name_("")
{
}

Parameter::Parameter(const std::string & name)
: name_(name), value_()
{
}

Parameter::Parameter(const std::string & name, const rclcpp::ParameterValue & value)
: name_(name), value_(value)
{
}

Parameter::Parameter(const rclcpp::node_interfaces::ParameterInfo & parameter_info)
: Parameter(parameter_info.descriptor.name, parameter_info.value)
{
}

bool
Parameter::operator==(const Parameter & rhs) const
{
  return this->name_ == rhs.name_ && this->value_ == rhs.value_;
}

bool
Parameter::operator!=(const Parameter & rhs) const
{
  return !(*this == rhs);
}

ParameterType
Parameter::get_type() const
{
  return value_.get_type();
}

std::string
Parameter::get_type_name() const
{
  return rclcpp::to_string(get_type());
}

const std::string &
Parameter::get_name() const
{
  return name_;
}

rcl_interfaces::msg::ParameterValue
Parameter::get_value_message() const
{
  return value_.to_value_msg();
}

const rclcpp::ParameterValue &
Parameter::get_parameter_value() const
{
  return value_;
}

bool
Parameter::as_bool() const
{
  return get_value<ParameterType::PARAMETER_BOOL>();
}

int64_t
Parameter::as_int() const
{
  return get_value<ParameterType::PARAMETER_INTEGER>();
}

double
Parameter::as_double() const
{
  return get_value<ParameterType::PARAMETER_DOUBLE>();
}

const std::string &
Parameter::as_string() const
{
  return get_value<ParameterType::PARAMETER_STRING>();
}

const std::vector<uint8_t> &
Parameter::as_byte_array() const
{
  return get_value<ParameterType::PARAMETER_BYTE_ARRAY>();
}

const std::vector<bool> &
Parameter::as_bool_array() const
{
  return get_value<ParameterType::PARAMETER_BOOL_ARRAY>();
}

const std::vector<int64_t> &
Parameter::as_integer_array() const
{
  return get_value<ParameterType::PARAMETER_INTEGER_ARRAY>();
}

const std::vector<double> &
Parameter::as_double_array() const
{
  return get_value<ParameterType::PARAMETER_DOUBLE_ARRAY>();
}

const std::vector<std::string> &
Parameter::as_string_array() const
{
  return get_value<ParameterType::PARAMETER_STRING_ARRAY>();
}

Parameter
Parameter::from_parameter_msg(const rcl_interfaces::msg::Parameter & parameter)
{
  return Parameter(parameter.name, parameter.value);
}

rcl_interfaces::msg::Parameter
Parameter::to_parameter_msg() const
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name_;
  parameter.value = value_.to_value_msg();
  return parameter;
}

std::string
Parameter::value_to_string() const
{
  return rclcpp::to_string(value_);
}

std::string
rclcpp::_to_json_dict_entry(const Parameter & param)
{
  std::stringstream ss;
  ss << "\"" << param.get_name() << "\": ";
  ss << "{\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

std::ostream &
rclcpp::operator<<(std::ostream & os, const rclcpp::Parameter & pv)
{
  os << std::to_string(pv);
  return os;
}

std::ostream &
rclcpp::operator<<(std::ostream & os, const std::vector<Parameter> & parameters)
{
  os << std::to_string(parameters);
  return os;
}

std::string
std::to_string(const rclcpp::Parameter & param)
{
  std::stringstream ss;
  ss << "{\"name\": \"" << param.get_name() << "\", ";
  ss << "\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

std::string
std::to_string(const std::vector<rclcpp::Parameter> & parameters)
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
    ss << rclcpp::_to_json_dict_entry(pv);
  }
  ss << "}";
  return ss.str();
}
