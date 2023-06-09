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

#ifndef RCLCPP_COMPONENTS__NODE_FACTORY_HPP__
#define RCLCPP_COMPONENTS__NODE_FACTORY_HPP__

#include "rclcpp_components/node_instance_wrapper.hpp"

namespace rclcpp_components
{

/// The NodeFactory interface is used by the class loader to instantiate components.
/**
 * The NodeFactory interface serves two purposes:
 *  * It allows for classes not derived from `rclcpp::Node` to be used as components.
 *  * It allows derived constructors to be called when components are loaded.
 */
class NodeFactory
{
public:
  NodeFactory() = default;

  virtual ~NodeFactory() = default;

  /// Create an instance of a component
  /**
   * \param[in] options Additional options used in the construction of the component.
   */
  virtual
  NodeInstanceWrapper
  create_node_instance(const rclcpp::NodeOptions & options) = 0;
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_FACTORY_HPP__
