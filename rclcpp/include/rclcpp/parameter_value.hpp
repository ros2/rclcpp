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

#ifndef RCLCPP__PARAMETER_VALUE_HPP_
#define RCLCPP__PARAMETER_VALUE_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

enum ParameterType : uint8_t
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

/// Return the name of a parameter type
RCLCPP_PUBLIC
std::string
to_string(ParameterType type);

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, ParameterType type);

/// Indicate the parameter type does not match the expected type.
class ParameterTypeException : public std::runtime_error
{
public:
  /// Construct an instance.
  /**
   * \param[in] expected the expected parameter type.
   * \param[in] actual the actual parameter type.
   */
  RCLCPP_PUBLIC
  ParameterTypeException(ParameterType expected, ParameterType actual)
  : std::runtime_error("expected [" + to_string(expected) + "] got [" + to_string(actual) + "]")
  {}
};

/// Store the type and value of a parameter.
class ParameterValue
{
public:
  /// Construct a parameter value with type PARAMETER_NOT_SET.
  RCLCPP_PUBLIC
  ParameterValue();
  /// Construct a parameter value from a message.
  RCLCPP_PUBLIC
  explicit ParameterValue(const rcl_interfaces::msg::ParameterValue & value);
  /// Construct a parameter value with type PARAMETER_BOOL.
  RCLCPP_PUBLIC
  explicit ParameterValue(const bool bool_value);
  /// Construct a parameter value with type PARAMETER_INTEGER.
  RCLCPP_PUBLIC
  explicit ParameterValue(const int int_value);
  /// Construct a parameter value with type PARAMETER_INTEGER.
  RCLCPP_PUBLIC
  explicit ParameterValue(const int64_t int_value);
  /// Construct a parameter value with type PARAMETER_DOUBLE.
  RCLCPP_PUBLIC
  explicit ParameterValue(const float double_value);
  /// Construct a parameter value with type PARAMETER_DOUBLE.
  RCLCPP_PUBLIC
  explicit ParameterValue(const double double_value);
  /// Construct a parameter value with type PARAMETER_STRING.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::string & string_value);
  /// Construct a parameter value with type PARAMETER_STRING.
  RCLCPP_PUBLIC
  explicit ParameterValue(const char * string_value);
  /// Construct a parameter value with type PARAMETER_BYTE_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<uint8_t> & byte_array_value);
  /// Construct a parameter value with type PARAMETER_BOOL_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<bool> & bool_array_value);
  /// Construct a parameter value with type PARAMETER_INTEGER_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<int> & int_array_value);
  /// Construct a parameter value with type PARAMETER_INTEGER_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<int64_t> & int_array_value);
  /// Construct a parameter value with type PARAMETER_DOUBLE_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<float> & double_array_value);
  /// Construct a parameter value with type PARAMETER_DOUBLE_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<double> & double_array_value);
  /// Construct a parameter value with type PARAMETER_STRING_ARRAY.
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<std::string> & string_array_value);

  /// Return an enum indicating the type of the set value.
  RCLCPP_PUBLIC
  ParameterType
  get_type() const;

  /// Return a message populated with the parameter value
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterValue
  to_value_msg() const;

  /// Equal operator.
  RCLCPP_PUBLIC
  bool
  operator==(const ParameterValue & rhs) const;

  /// Not equal operator.
  RCLCPP_PUBLIC
  bool
  operator!=(const ParameterValue & rhs) const;

  // The following get() variants require the use of ParameterType

  template<ParameterType type>
  constexpr
  typename std::enable_if<type == ParameterType::PARAMETER_BOOL, const bool &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      throw ParameterTypeException(ParameterType::PARAMETER_BOOL, get_type());
    }
    return value_.bool_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<type == ParameterType::PARAMETER_INTEGER, const int64_t &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      throw ParameterTypeException(ParameterType::PARAMETER_INTEGER, get_type());
    }
    return value_.integer_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<type == ParameterType::PARAMETER_DOUBLE, const double &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      throw ParameterTypeException(ParameterType::PARAMETER_DOUBLE, get_type());
    }
    return value_.double_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<type == ParameterType::PARAMETER_STRING, const std::string &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      throw ParameterTypeException(ParameterType::PARAMETER_STRING, get_type());
    }
    return value_.string_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<
    type == ParameterType::PARAMETER_BYTE_ARRAY, const std::vector<uint8_t> &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_BYTE_ARRAY, get_type());
    }
    return value_.byte_array_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<
    type == ParameterType::PARAMETER_BOOL_ARRAY, const std::vector<bool> &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_BOOL_ARRAY, get_type());
    }
    return value_.bool_array_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<
    type == ParameterType::PARAMETER_INTEGER_ARRAY, const std::vector<int64_t> &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_INTEGER_ARRAY, get_type());
    }
    return value_.integer_array_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<
    type == ParameterType::PARAMETER_DOUBLE_ARRAY, const std::vector<double> &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_DOUBLE_ARRAY, get_type());
    }
    return value_.double_array_value;
  }

  template<ParameterType type>
  constexpr
  typename std::enable_if<
    type == ParameterType::PARAMETER_STRING_ARRAY, const std::vector<std::string> &>::type
  get() const
  {
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_STRING_ARRAY, get_type());
    }
    return value_.string_array_value;
  }

  // The following get() variants allow the use of primitive types

  template<typename type>
  constexpr
  typename std::enable_if<std::is_same<type, bool>::value, const bool &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_BOOL>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_integral<type>::value && !std::is_same<type, bool>::value, const int64_t &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_INTEGER>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<std::is_floating_point<type>::value, const double &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_DOUBLE>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<std::is_convertible<type, std::string>::value, const std::string &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_STRING>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<uint8_t> &>::value, const std::vector<uint8_t> &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_BYTE_ARRAY>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<bool> &>::value, const std::vector<bool> &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_BOOL_ARRAY>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<int64_t> &>::value, const std::vector<int64_t> &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_INTEGER_ARRAY>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<double> &>::value, const std::vector<double> &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_DOUBLE_ARRAY>();
  }

  template<typename type>
  constexpr
  typename std::enable_if<
    std::is_convertible<
      type, const std::vector<std::string> &>::value, const std::vector<std::string> &>::type
  get() const
  {
    return get<ParameterType::PARAMETER_STRING_ARRAY>();
  }

private:
  rcl_interfaces::msg::ParameterValue value_;
};

/// Return the value of a parameter as a string
RCLCPP_PUBLIC
std::string
to_string(const ParameterValue & type);

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_VALUE_HPP_
