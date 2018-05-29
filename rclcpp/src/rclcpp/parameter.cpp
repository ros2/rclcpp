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
}

ParameterVariant::ParameterVariant(const std::string & name, const ParameterValue & value)
: name_(name), value_(value)
{
}

ParameterType
ParameterVariant::get_type() const
{
  return value_.get_type();
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
ParameterVariant::get_value_message() const
{
  return value_.get_message();
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
  return ParameterVariant(parameter.name, parameter.value);
}

rcl_interfaces::msg::Parameter
ParameterVariant::to_parameter()
{
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name_;
  parameter.value = value_.get_message();
  return parameter;
}

std::string
ParameterVariant::value_to_string() const
{
  return rclcpp::to_string(value_);
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
