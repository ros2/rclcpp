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

#ifndef COMPONENT_MANAGER_HPP__
#define COMPONENT_MANAGER_HPP__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

class ComponentManagerException : public std::runtime_error
{
public:
  explicit ComponentManagerException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

class ComponentManager : public rclcpp::Node
{
public:
  using LoadNode = composition_interfaces::srv::LoadNode;
  using UnloadNode = composition_interfaces::srv::UnloadNode;
  using ListNodes = composition_interfaces::srv::ListNodes;

  /// Represents a component resource.
  /**
   * Is a pair of class name (for class loader) and library path (absolute)
   */
  using ComponentResource = std::pair<std::string, std::string>;

  ComponentManager(
    std::weak_ptr<rclcpp::executor::Executor> executor);

  ~ComponentManager();

  /// Return a list of valid loadable components in a given package.
  std::vector<ComponentResource>
  get_component_resources(const std::string & package_name) const;

  std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource & resource);

private:
  void
  OnLoadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response);

  void
  OnUnloadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response);

  void
  OnListNodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response);

private:
  std::weak_ptr<rclcpp::executor::Executor> executor_;

  uint64_t unique_id {1};
  std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders_;
  std::map<uint64_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;

  rclcpp::Service<LoadNode>::SharedPtr loadNode_srv_;
  rclcpp::Service<UnloadNode>::SharedPtr unloadNode_srv_;
  rclcpp::Service<ListNodes>::SharedPtr listNodes_srv_;
};

}  // namespace rclcpp_components

#endif  // COMPONENT_MANAGER_HPP__
