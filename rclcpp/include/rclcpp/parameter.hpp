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
namespace parameter
{

// Structure to store an arbitrary parameter with templated get/set methods
class ParameterVariant
{
public:
  RCLCPP_PUBLIC
  ParameterVariant();

  RCLCPP_PUBLIC
  ParameterVariant(const std::string & name, const ParameterValue & value);

  template<typename ValueTypeT>
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, ValueTypeT value)
  : ParameterVariant(name, ParameterValue(value))
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
  get_parameter_value() const;

  /// Get value of parameter using rclcpp::ParameterType as template argument.
  template<ParameterType ParamT>
  decltype(ParameterValue().get<ParamT>())
  get_value() const
  {
    return value_.get<ParamT>();
  }

  /// Get value of parameter using c++ types as template argument
  template<typename T>
  decltype(ParameterValue().get<T>())
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
  static ParameterVariant
  from_parameter(const rcl_interfaces::msg::Parameter & parameter);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::Parameter
  to_parameter();

  RCLCPP_PUBLIC
  std::string
  value_to_string() const;

private:
  template<typename ValType, typename PrintType = ValType>
  std::string
  array_to_string(
    const std::vector<ValType> & array,
    const std::ios::fmtflags format_flags = std::ios::dec) const
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

  std::string name_;
  ParameterValue value_;
};

/// Return a json encoded version of the parameter intended for a dict.
RCLCPP_PUBLIC
std::string
_to_json_dict_entry(const ParameterVariant & param);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const rclcpp::parameter::ParameterVariant & pv);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const std::vector<ParameterVariant> & parameters);

}  // namespace parameter
}  // namespace rclcpp

namespace std
{

/// Return a json encoded version of the parameter intended for a list.
RCLCPP_PUBLIC
std::string
to_string(const rclcpp::parameter::ParameterVariant & param);

/// Return a json encoded version of a vector of parameters, as a string.
RCLCPP_PUBLIC
std::string
to_string(const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

}  // namespace std

#endif  // RCLCPP__PARAMETER_HPP_
