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
 *
 * // declare statically typed parameter, no default value given.
 * node.declare_parameter(
 *   "my_int", rclcpp::ParameterType::INTEGER_PARAMETER);
 *
 * // declare statically typed parameter, type inferred from given default, with descriptor.
 * rcl_interfaces::msg::ParameterDescriptor descriptor;
 * //>> Set descriptor <<
 * node.declare_parameter("my_int", 5, descriptor);
 *
 * // declare statically typed parameter, type inferred from descriptor.
 * rcl_interfaces::msg::ParameterDescriptor descriptor;
 * descriptor.allowed_type = static_cast<uint8_t>(rclcpp::ParameterType::INTEGER_PARAMETER);
 * //>> Set other things in descriptor.<<
 * node.declare_parameter("my_int", {descriptor, false});
 *
 * // declare parameter that can change of type
 * node.declare_parameter("my_param", rclcpp::ParameterType::PARAMETER_NOT_SET);
 *
 * // declare parameter that can change of type with default value
 * node.declare_parameter("my_param", 5, rclcpp::ParameterDescriptor{false});
 *
 * // declare parameter that can change of type with default value and descriptor
 * rcl_interfaces::msg::ParameterDescriptor descriptor;
 * //>> Set descriptor <<
 * node.declare_parameter("my_param", 5, {descriptor, false});
 */
class ParameterDescriptor
{
public:
  /// Default constructor.
  explicit ParameterDescriptor(bool infer_type_from_value = true)
  : infer_type_from_value_{infer_type_from_value} {}

  /// Implicit constructor from parameter descriptor message.
  ParameterDescriptor(
    const rcl_interfaces::msg::ParameterDescriptor & msg, bool infer_type_from_value = true)
  // NOLINT: implicit conversion constructor
  : impl_(msg), infer_type_from_value_{infer_type_from_value} {}

  /// Getter of internal message representation.
  rcl_interfaces::msg::ParameterDescriptor &
  get_msg()
  {return impl_;}

  /// Getter of internal message representation.
  const rcl_interfaces::msg::ParameterDescriptor &
  get_msg() const
  {return impl_;}

  /// Return true if we have to ignore the allowed type provided in the descriptor and infer
  /// it later from the default value.
  bool
  do_infer_type_from_value() const
  {return infer_type_from_value_;}

private:
  rcl_interfaces::msg::ParameterDescriptor impl_;
  bool infer_type_from_value_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_DESCRIPTOR_HPP_
