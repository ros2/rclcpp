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

#ifndef RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__
#define RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__

#include <functional>
#include <memory>

#include "rclcpp/node_interfaces/node_base_interface.hpp"

namespace rclcpp_components
{
/// The NodeInstanceWrapper encapsulates the node instance.
class NodeInstanceWrapper
{
public:
  using NodeBaseInterfaceGetter = std::function<
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr(const std::shared_ptr<void> &)>;

  NodeInstanceWrapper()
  : node_instance_(nullptr)
  {}

  NodeInstanceWrapper(
    std::shared_ptr<void> node_instance,
    NodeBaseInterfaceGetter node_base_interface_getter)
  : node_instance_(node_instance), node_base_interface_getter_(node_base_interface_getter)
  {}

  /// Get a type-erased pointer to the original Node instance
  /**
   * This is only for debugging and special cases.
   * For most cases `get_node_base_interface` will be sufficient.
   *
   * \return Shared pointer to the encapsulated Node instance.
   */
  const std::shared_ptr<void>
  get_node_instance() const
  {
    return node_instance_;
  }

  /// Get NodeBaseInterface pointer for the encapsulated Node Instance.
  /**
   * \return Shared NodeBaseInterface pointer of the encapsulated Node instance.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface()
  {
    return node_base_interface_getter_(node_instance_);
  }

private:
  std::shared_ptr<void> node_instance_;
  NodeBaseInterfaceGetter node_base_interface_getter_;
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__
