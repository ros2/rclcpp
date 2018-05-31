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

#ifndef RCLCPP__PARAMETER_HPP_
#define RCLCPP__PARAMETER_HPP_

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Structure to store an arbitrary parameter with templated get/set methods.
class Parameter
{
public:
  RCLCPP_PUBLIC
  Parameter();

  RCLCPP_PUBLIC
  Parameter(const std::string & name, const ParameterValue & value);

  template<typename ValueTypeT>
  explicit Parameter(const std::string & name, ValueTypeT value)
  : Parameter(name, ParameterValue(value))
  {
  }

  RCLCPP_PUBLIC
  ParameterType
  get_type() const;

  RCLCPP_PUBLIC
  std::string
  get_type_name() const;

  RCLCPP_PUBLIC
  const std::string &
  get_name() const;

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterValue
  get_value_message() const;

  /// Get value of parameter using rclcpp::ParameterType as template argument.
  template<ParameterType ParamT>
  decltype(auto)
  get_value() const
  {
    return value_.get<ParamT>();
  }

  /// Get value of parameter using c++ types as template argument.
  template<typename T>
  decltype(auto)
  get_value() const
  {
    return value_.get<T>();
  }

  RCLCPP_PUBLIC
  bool
  as_bool() const;

  RCLCPP_PUBLIC
  int64_t
  as_int() const;

  RCLCPP_PUBLIC
  double
  as_double() const;

  RCLCPP_PUBLIC
  const std::string &
  as_string() const;

  RCLCPP_PUBLIC
  const std::vector<uint8_t> &
  as_byte_array() const;

  RCLCPP_PUBLIC
  const std::vector<bool> &
  as_bool_array() const;

  RCLCPP_PUBLIC
  const std::vector<int64_t> &
  as_integer_array() const;

  RCLCPP_PUBLIC
  const std::vector<double> &
  as_double_array() const;

  RCLCPP_PUBLIC
  const std::vector<std::string> &
  as_string_array() const;

  RCLCPP_PUBLIC
  static Parameter
  from_parameter_msg(const rcl_interfaces::msg::Parameter & parameter);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::Parameter
  to_parameter_msg() const;

  RCLCPP_PUBLIC
  std::string
  value_to_string() const;

private:
  std::string name_;
  ParameterValue value_;
};

/// Return a json encoded version of the parameter intended for a dict.
RCLCPP_PUBLIC
std::string
_to_json_dict_entry(const Parameter & param);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const rclcpp::Parameter & pv);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const std::vector<Parameter> & parameters);

}  // namespace rclcpp

namespace std
{

/// Return a json encoded version of the parameter intended for a list.
RCLCPP_PUBLIC
std::string
to_string(const rclcpp::Parameter & param);

/// Return a json encoded version of a vector of parameters, as a string.
RCLCPP_PUBLIC
std::string
to_string(const std::vector<rclcpp::Parameter> & parameters);

}  // namespace std

#endif  // RCLCPP__PARAMETER_HPP_
