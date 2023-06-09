// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__
#define RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__

#include "class_loader/class_loader.hpp"
#include "rclcpp_components/node_factory_template.hpp"

/// Register a component that can be dynamically loaded at runtime.
/**
 * The registration macro should appear once per component per library.
 * The macro should appear in a single translation unit.
 *
 * Valid arguments for NodeClass shall:
 *  * Have a constructor that takes a single argument that is a `rclcpp::NodeOptions` instance.
 *  * Have a method of of the signature:
 *      `rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface`
 *
 * Note: NodeClass does not need to inherit from `rclcpp::Node`, but it is the easiest way.
 */
#define RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass) \
  CLASS_LOADER_REGISTER_CLASS( \
    rclcpp_components::NodeFactoryTemplate<NodeClass>, \
    rclcpp_components::NodeFactory)

#endif  // RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__
