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

#ifndef RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__
#define RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "composition_interfaces/srv/list_nodes.hpp"

#include "rclcpp_components/node_factory.hpp"

namespace rclcpp_components
{

class ComponentManager : public rclcpp::Node
{
public:
  using LoadNode = composition_interfaces::srv::LoadNode;
  using UnloadNode = composition_interfaces::srv::UnloadNode;
  using ListNodes = composition_interfaces::srv::ListNodes;

  ComponentManager(rclcpp::executor::Executor * executor);

private:
  void OnLoadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response);

  void OnUnloadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response);

  void OnListNodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response);

private:
  rclcpp::executor::Executor * executor_;

  std::map<uint64_t, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes_;
  std::map<std::string, class_loader::ClassLoader *> loaders_;

  uint64_t unique_id {1};

  rclcpp::Service<LoadNode>::SharedPtr loadNode_srv_;
  rclcpp::Service<UnloadNode>::SharedPtr unloadNode_srv_;
  rclcpp::Service<ListNodes>::SharedPtr listNodes_srv_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__
