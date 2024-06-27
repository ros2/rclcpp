// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_
#define RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_

// C++ Standard library includes
#include <functional>
#include <utility>
#include <memory>
#include <string>

// Additional ROS libraries needed
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "node_interfaces/node_parameters_interface.hpp"

namespace rclcpp
{

// Implements ParameterDesription class with builder design pattern
class ParameterDescription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterDescription)

  // List of classes the builder manages
  RCLCPP_PUBLIC
  ParameterDescription();

  // Our Main build methods which will construct the base class
  RCLCPP_PUBLIC rcl_interfaces::msg::ParameterDescriptor build() const;

  // Builder Methods:
  // Describes the instances in a parameter_description object
  RCLCPP_PUBLIC ParameterDescription & set_name(const std::string & name);
  RCLCPP_PUBLIC ParameterDescription & set_type(std::uint8_t type);
  RCLCPP_PUBLIC ParameterDescription & set_description_text(const std::string & description);
  RCLCPP_PUBLIC ParameterDescription & set_additional_constraints(const std::string & constraints);
  RCLCPP_PUBLIC ParameterDescription & set_read_only(bool read_only);
  RCLCPP_PUBLIC ParameterDescription & set_dynamic_typing(bool dynamic_typing);
  RCLCPP_PUBLIC ParameterDescription & set_floating_point_description_range(
    float min, float max, float step);
  RCLCPP_PUBLIC ParameterDescription & set_integer_description_range(int min, int max, int step);

  // Need the current node in order to begin the configuration state
  // for it via the declare_parameter function
  template<typename NodeT>
  ParameterDescription & declare_parameter(
    const rclcpp::ParameterValue & default_value,
    NodeT && node)
  {
    auto node_param = rclcpp::node_interfaces::get_node_parameters_interface(node);
    node_param->declare_parameter(
      parameter_descriptor.name, default_value,
      parameter_descriptor);
    return *this;
  }

private:
  // The main descriptor object we're meant to initialize and adjust
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_
