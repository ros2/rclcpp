// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PARAMETER_DESCRIPTOR_HPP_
#define RCLCPP__PARAMETER_DESCRIPTOR_HPP_

namespace rclcpp
{

/// rcl_interfaces::msg::ParameterDescriptor wrapper used to streamline parameter declarations.
/**
 * Examples:
 * ```cpp
 * // declare statically typed parameter, type inferred from gived default value
 * node.declare_parameter("my_int", 5);
 * // declare statically typed parameter, no default value given.
 * node.declare_parameter("my_int", rclcpp::ParameterType::INTEGER_PARAMETER);
 * // declare statically typed parameter, no default value given.
 * node.declare_parameter(my_int, rclcpp::ParameterValue(), rclcpp::ParameterType::INTEGER_PARAMETER);
 */
class ParameterDescriptor
{
public:
  /// Default constructor.
  explicit ParameterDescriptor(bool static_typing = true) : static_typing_{static_typing} {}

  /// Implicit constructor from parameter descriptor message.
  ParameterDescriptor(  // NOLINT: implicit conversion constructor
    const rcl_interfaces::msg::ParameterDescriptor & msg)
  : impl_(msg) {}

  /// Implicit constructor from a parameter type.
  ParameterDescriptor(rclcpp::ParameterType type)  // NOLINT: implicit conversion constructor
  : static_typing_{type != rclcpp::ParameterType::PARAMETER_NOT_SET} {impl_.allowed_type = type;}

  /// Getter of internal message representation.
  rcl_interfaces::msg::ParameterDescriptor &
  get_msg()
  {return impl_;}

  /// Getter of internal message representation.
  const rcl_interfaces::msg::ParameterDescriptor &
  get_msg() const
  {return impl_;}

  /// Return true if the parameter cannot change its type, false otherwise.
  bool
  is_statically_typed() const
  {return static_typing_ || impl_.allowed_type != rclcpp::ParameterType::PARAMETER_NOT_SET;}

private:
  rcl_interfaces::msg::ParameterDescriptor impl_;
  bool static_typing_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_DESCRIPTOR_HPP_
