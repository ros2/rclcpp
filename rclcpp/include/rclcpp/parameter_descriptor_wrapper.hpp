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
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "macros.hpp"
#include "rclcpp/node.hpp"
#include "node_interfaces/node_base_interface.hpp"
#include "node_interfaces/node_base.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp"

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

  // Need the current node in order to begin the configuration state
  // for it via the declare_parameter function which setups the Node
  template<typename ParameterType>
  ParameterDescription & DeclareParameter(
    ParameterType default_value,
    rclcpp::Node::SharedPtr required_node_ptr)
  {
    required_node_ptr->declare_parameter<ParameterType>(
      parameter_descriptor.name, default_value,
      parameter_descriptor);
    return *this;
  }

  template<typename ParameterType>
  ParameterDescription & DeclareParameter(
    ParameterType default_value,
    rclcpp_lifecycle::LifecycleNode::SharedPtr required_node_ptr)
  {
    required_node_ptr->declare_parameter<ParameterType>(
      parameter_descriptor.name, default_value,
      parameter_descriptor);
    return *this;
  }

  // Simplification Methods:
  // The user should be able to set up ranges easily, they should then be able to call a function
  // this function sets up the specifics for this class
  // Here we will have a difference between generic types and templated types
  ParameterDescription & SetFloatingPointDescriptionRange(
    float min = 0.0f, float max = 1.0f,
    float step = 0.0f);
  // We will again need access to the current development node to declare its parameters
  template<typename ParameterType>
  ParameterDescription & SetFloatingPointDescriptionRange(
    rclcpp::Node::SharedPtr currentNode, const std::string & name,
    ParameterType default_value, float min = 0.0f,
    float max = 1.0f, float step = 0.0f)
  {
    parameter_descriptor.floating_point_range.resize(1);
    parameter_descriptor.floating_point_range.at(0).from_value = min;
    parameter_descriptor.floating_point_range.at(0).to_value = max;
    parameter_descriptor.floating_point_range.at(0).step = step;

    // For this function we can outright declare the parameters using the specified type
    currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

    return *this;
  }

  template<typename ParameterType>
  ParameterDescription & SetFloatingPointDescriptionRange(
    rclcpp_lifecycle::LifecycleNode::SharedPtr currentNode,
    const std::string & name, ParameterType default_value,
    float min = 0.0f, float max = 1.0f,
    float step = 0.0f)
  {
    parameter_descriptor.floating_point_range.resize(1);
    parameter_descriptor.floating_point_range.at(0).from_value = min;
    parameter_descriptor.floating_point_range.at(0).to_value = max;
    parameter_descriptor.floating_point_range.at(0).step = step;

    // For this function we can outright declare the parameters using the specified type
    currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

    return *this;
  }

  ParameterDescription & SetIntegerDescriptionRange(int min = 0, int max = 1, int step = 0);
  // We will again need access to the current development node to declare its parameters
  template<typename ParameterType>
  ParameterDescription & SetIntegerDescriptionRange(
    rclcpp::Node::SharedPtr currentNode, const std::string & name,
    ParameterType default_value, int min = 0,
    int max = 1, int step = 0)
  {
    parameter_descriptor.integer_range.resize(1);
    parameter_descriptor.integer_range.at(0).from_value = min;
    parameter_descriptor.integer_range.at(0).to_value = max;
    parameter_descriptor.integer_range.at(0).step = step;

    // For this function we can outright declare the parameters using the specified type
    currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

    return *this;
  }

  // We will again need access to the current development node to declare its parameters
  template<typename ParameterType>
  ParameterDescription & SetIntegerDescriptionRange(
    rclcpp_lifecycle::LifecycleNode currentNode, const std::string & name,
    ParameterType default_value, int min = 0,
    int max = 1, int step = 0)
  {
    parameter_descriptor.integer_range.resize(1);
    parameter_descriptor.integer_range.at(0).from_value = min;
    parameter_descriptor.integer_range.at(0).to_value = max;
    parameter_descriptor.integer_range.at(0).step = step;

    // For this function we can outright declare the parameters using the specified type
    currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

    return *this;
  }

private:
  // The main descriptor object we're meant to initialize and adjust
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = {};
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_
