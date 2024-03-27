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
#include "node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"

namespace rclcpp
{

// Implements ParameterDesription class with builder design pattern
class ParameterDescription
{
public:
  // List of classes the builder manages
  ParameterDescription();

  // Our Main build methods which will construct the base class
  rcl_interfaces::msg::ParameterDescriptor build() const;

  // Builder Methods:
  // Describes the instances in a parameter_description object
  ParameterDescription & SetName(const std::string & name);
  ParameterDescription & SetType(std::uint8_t type);
  ParameterDescription & SetDescriptionText(const std::string & description);
  ParameterDescription & SetAdditionalConstraints(const std::string & constraints);
  ParameterDescription & SetReadOnly(bool read_only);
  ParameterDescription & SetDynamicTyping(bool dynamic_typing);
  ParameterDescription & SetFloatingPointDescriptionRange(float min = 0.0f, float max = 1.0f,
    float step = 0.0f);
  ParameterDescription & SetIntegerDescriptionRange(int min = 0, int max = 1, int step = 0);

  // Need the current node in order to begin the configuration state
  // for it via the declare_parameter function
  template<typename ParameterType, typename NodeT>
  ParameterDescription & DeclareParameter(
    ParameterType default_value,
    NodeT && node)
  {
    auto node_param = rclcpp::node_interfaces::get_node_parameters_interface(node);
    node_param->declare_parameter<ParameterType>(
      parameter_descriptor.name, default_value,
      parameter_descriptor);
    return *this;
  }

private:
  // The main descriptor object we're meant to initialize and adjust
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = {};
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_
