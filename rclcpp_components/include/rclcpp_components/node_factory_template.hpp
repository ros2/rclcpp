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

#ifndef RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__
#define RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__

#include <memory>

#include "rclcpp_components/node_factory.hpp"

namespace rclcpp_components
{

template<typename NodeT>
class NodeFactoryTemplate : public NodeFactory
{
public:
  NodeFactoryTemplate() = default;
  virtual ~NodeFactoryTemplate() = default;

  virtual
  NodeInstanceWrapper
  create_node_instance(rclcpp::NodeOptions options)
  {
    auto node = std::make_shared<NodeT>(options);

    return NodeInstanceWrapper(node,
             std::bind(&NodeT::get_node_base_interface, node));
  }
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__
