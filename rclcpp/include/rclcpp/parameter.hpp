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
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

class Parameter;

namespace node_interfaces
{
// Internal struct for holding useful info about parameters
struct ParameterInfo
{
  /// Current value of the parameter.
  rclcpp::ParameterValue value;

  /// A description of the parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
};
}  // namespace node_interfaces

namespace detail
{

// This helper function is required because you cannot do specialization on a
// class method, so instead we specialize this template function and call it
// from the unspecialized, but dependent, class method.
template<typename T>
auto
get_value_helper(const rclcpp::Parameter * parameter);

}  // namespace detail

/// Structure to store an arbitrary parameter with templated get/set methods.
class Parameter
{
public:
  /// Construct with an empty name and a parameter value of type rclcpp::PARAMETER_NOT_SET.
  RCLCPP_PUBLIC
  Parameter();

  /// Construct with given name and a parameter value of type rclcpp::PARAMETER_NOT_SET.
  RCLCPP_PUBLIC
  explicit Parameter(const std::string & name);

  /// Construct with given name and given parameter value.
  RCLCPP_PUBLIC
  Parameter(const std::string & name, const ParameterValue & value);

  /// Construct with given name and given parameter value.
  template<typename ValueTypeT>
  Parameter(const std::string & name, ValueTypeT value)
  : Parameter(name, ParameterValue(value))
  {}

  RCLCPP_PUBLIC
  explicit Parameter(const rclcpp::node_interfaces::ParameterInfo & parameter_info);

  /// Equal operator.
  RCLCPP_PUBLIC
  bool
  operator==(const Parameter & rhs) const;

  /// Not equal operator.
  RCLCPP_PUBLIC
  bool
  operator!=(const Parameter & rhs) const;

  /// Get the type of the parameter
  RCLCPP_PUBLIC
  ParameterType
  get_type() const;

  /// Get the type name of the parameter
  RCLCPP_PUBLIC
  std::string
  get_type_name() const;

  /// Get the name of the parameter
  RCLCPP_PUBLIC
  const std::string &
  get_name() const;

  /// Get value of parameter as a parameter message.
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterValue
  get_value_message() const;

  /// Get the internal storage for the parameter value.
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  get_parameter_value() const;

  /// Get value of parameter using rclcpp::ParameterType as template argument.
  /**
   * \throws rclcpp::exceptions::InvalidParameterTypeException if the type doesn't match
   */
  template<ParameterType ParamT>
  decltype(auto)
  get_value() const
  {
    return value_.get<ParamT>();
  }

  /// Get value of parameter using c++ types as template argument.
  template<typename T>
  decltype(auto)
  get_value() const;

  /// Get value of parameter as boolean.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  bool
  as_bool() const;

  /// Get value of parameter as integer.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  int64_t
  as_int() const;

  /// Get value of parameter as double.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  double
  as_double() const;

  /// Get value of parameter as string.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::string &
  as_string() const;

  /// Get value of parameter as byte array (vector<uint8_t>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<uint8_t> &
  as_byte_array() const;

  /// Get value of parameter as bool array (vector<bool>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<bool> &
  as_bool_array() const;

  /// Get value of parameter as integer array (vector<int64_t>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<int64_t> &
  as_integer_array() const;

  /// Get value of parameter as double array (vector<double>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<double> &
  as_double_array() const;

  /// Get value of parameter as string array (vector<std::string>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<std::string> &
  as_string_array() const;

  /// Convert a parameter message in a Parameter class object.
  RCLCPP_PUBLIC
  static Parameter
  from_parameter_msg(const rcl_interfaces::msg::Parameter & parameter);

  /// Convert the class in a parameter message.
  RCLCPP_PUBLIC
  rcl_interfaces::msg::Parameter
  to_parameter_msg() const;

  /// Get value of parameter as a string.
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

namespace detail
{

template<typename T>
auto
get_value_helper(const rclcpp::Parameter * parameter)
{
  return parameter->get_parameter_value().get<T>();
}

// Specialization allowing Parameter::get() to return a const ref to the parameter value object.
template<>
inline
auto
get_value_helper<rclcpp::ParameterValue>(const rclcpp::Parameter * parameter)
{
  return parameter->get_parameter_value();
}

// Specialization allowing Parameter::get() to return a const ref to the parameter itself.
template<>
inline
auto
get_value_helper<rclcpp::Parameter>(const rclcpp::Parameter * parameter)
{
  // Use this lambda to ensure it's a const reference being returned (and not a copy).
  auto type_enforcing_lambda =
    [&parameter]() -> const rclcpp::Parameter & {
      return *parameter;
    };
  return type_enforcing_lambda();
}

}  // namespace detail

template<typename T>
decltype(auto)
Parameter::get_value() const
{
  try {
    // use the helper to specialize for the ParameterValue and Parameter cases.
    return detail::get_value_helper<T>(this);
  } catch (const ParameterTypeException & ex) {
    throw exceptions::InvalidParameterTypeException(this->name_, ex.what());
  }
}

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
