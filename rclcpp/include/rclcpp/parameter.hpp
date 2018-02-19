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
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace parameter
{

enum ParameterType
{
  PARAMETER_NOT_SET = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
  PARAMETER_BOOL = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
  PARAMETER_INTEGER = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
  PARAMETER_DOUBLE = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
  PARAMETER_STRING = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
  PARAMETER_BYTE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,
  PARAMETER_BOOL_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
};

// Structure to store an arbitrary parameter with templated get/set methods
class ParameterVariant
{
public:
  RCLCPP_PUBLIC
  ParameterVariant();
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const bool bool_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const int int_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const int64_t int_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const float double_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const double double_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const std::string & string_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(const std::string & name, const char * string_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<uint8_t> & byte_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<bool> & bool_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<int> & int_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<int64_t> & int_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<float> & double_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<double> & double_array_value);
  RCLCPP_PUBLIC
  explicit ParameterVariant(
    const std::string & name,
    const std::vector<std::string> & string_array_value);

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

  // The following get_value() variants require the use of ParameterType

  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_BOOL, bool>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.bool_value;
  }

  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_INTEGER, int64_t>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.integer_value;
  }

  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_DOUBLE, double>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.double_value;
  }

  template<ParameterType type>
  typename std::enable_if<type == ParameterType::PARAMETER_STRING, const std::string &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.string_value;
  }

  template<ParameterType type>
  typename std::enable_if<
    type == ParameterType::PARAMETER_BYTE_ARRAY, const std::vector<uint8_t> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.byte_array_value;
  }

  template<ParameterType type>
  typename std::enable_if<
    type == ParameterType::PARAMETER_BOOL_ARRAY, const std::vector<bool> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.bool_array_value;
  }

  template<ParameterType type>
  typename std::enable_if<
    type == ParameterType::PARAMETER_INTEGER_ARRAY, const std::vector<int64_t> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.integer_array_value;
  }

  template<ParameterType type>
  typename std::enable_if<
    type == ParameterType::PARAMETER_DOUBLE_ARRAY, const std::vector<double> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.double_array_value;
  }

  template<ParameterType type>
  typename std::enable_if<
    type == ParameterType::PARAMETER_STRING_ARRAY, const std::vector<std::string> &>::type
  get_value() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
      // TODO(wjwwood): use custom exception
      throw std::runtime_error("Invalid type");
    }
    return value_.string_array_value;
  }

  // The following get_value() variants allow the use of primitive types

  template<typename type>
  typename std::enable_if<std::is_same<type, bool>::value, bool>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_BOOL>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_integral<type>::value && !std::is_same<type, bool>::value, int64_t>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_INTEGER>();
  }

  template<typename type>
  typename std::enable_if<std::is_floating_point<type>::value, double>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_DOUBLE>();
  }

  template<typename type>
  typename std::enable_if<std::is_convertible<type, std::string>::value, const std::string &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_STRING>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<uint8_t> &>::value, const std::vector<uint8_t> &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_BYTE_ARRAY>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<bool> &>::value, const std::vector<bool> &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_BOOL_ARRAY>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<int64_t> &>::value, const std::vector<int64_t> &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_INTEGER_ARRAY>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<double> &>::value, const std::vector<double> &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_DOUBLE_ARRAY>();
  }

  template<typename type>
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<std::string> &>::value, const std::vector<std::string> &>::type
  get_value() const
  {
    return get_value<ParameterType::PARAMETER_STRING_ARRAY>();
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

  std::string value_to_string() const;

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

  template<typename OutputType, typename InputType>
  void vector_assign(OutputType & output, const InputType & input)
  {
    output.assign(input.begin(), input.end());
  }

  std::string name_;
  rcl_interfaces::msg::ParameterValue value_;
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
