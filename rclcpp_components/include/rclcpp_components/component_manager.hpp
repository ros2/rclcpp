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

/** \mainpage rclcpp_components: Package containing tools for dynamically loadable components.
 *
 * - ComponentManager: Node to manage components. It has the services to load, unload and list
 *   current components.
 *   - rclcpp_components/component_manager.hpp)
 * - Node factory: The NodeFactory interface is used by the class loader to instantiate components.
 *   - rclcpp_components/node_factory.hpp)
 *   - It allows for classes not derived from `rclcpp::Node` to be used as components.
 *   - It allows derived constructors to be called when components are loaded.
 *
 * Some useful abstractions and utilities:
 * - [RCLCPP_COMPONENTS_REGISTER_NODE: Register a component that can be dynamically loaded
 *   at runtime.
 *   - (include/rclcpp_components/register_node_macro.hpp)
 *
 * Some useful internal abstractions and utilities:
 * - Macros for controlling symbol visibility on the library
 *   - rclcpp_components/visibility_control.h
 *
 * Package containing CMake tools for register components:
 * - `rclcpp_components_register_node` Register an rclcpp component with the ament resource index
 *    and create an executable.
 * - `rclcpp_components_register_nodes` Register an rclcpp component with the ament resource index.
 *    The passed library can contain multiple nodes each registered via macro.
 */

#ifndef RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__
#define RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "composition_interfaces/srv/list_nodes.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/visibility_control.hpp"

namespace class_loader
{
class ClassLoader;
}  // namespace class_loader

namespace rclcpp_components
{

/// Thrown when an error happens in the component Manager class.
class ComponentManagerException : public std::runtime_error
{
public:
  explicit ComponentManagerException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

/// ComponentManager handles the services to load, unload, and get the list of loaded components.
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

  /// Default constructor
  /**
   * Initializes the component manager. It creates the services: load node, unload node
   * and list nodes.
   *
   * \param executor the executor which will spin the node.
   * \param node_name the name of the node that the data originates from.
   * \param node_options additional options to control creation of the node.
   */
  RCLCPP_COMPONENTS_PUBLIC
  ComponentManager(
    std::weak_ptr<rclcpp::Executor> executor,
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  RCLCPP_COMPONENTS_PUBLIC
  virtual ~ComponentManager();

  /// Return a list of valid loadable components in a given package.
  /*
   * \param package_name name of the package
   * \param resource_index name of the executable
   * \throws ComponentManagerException if the resource was not found or a invalid resource entry
   * \return a list of component resources
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual std::vector<ComponentResource>
  get_component_resources(
    const std::string & package_name,
    const std::string & resource_index = "rclcpp_components") const;

  /// Instantiate a component from a dynamic library.
  /*
   * \param resource a component resource (class name + library path)
   * \return a NodeFactory interface
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource & resource);

protected:
  /// Service callback to load a new node in the component
  /*
   * This function allows to add parameters, remap rules, a specific node, name a namespace
   * and/or additional arguments.
   *
   * \param request_header unused
   * \param request information with the node to load
   * \param response
   * \throws std::overflow_error if node_id suffers an overflow. Very unlikely to happen at 1 kHz
   *   (very optimistic rate). it would take 585 years.
   * \throws ComponentManagerException In the case that the component constructor throws an
   *   exception, rethrow into the following catch block.
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void
  OnLoadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response);

  /// Service callback to unload a node in the component
  /*
   * \param request_header unused
   * \param request unique identifier to remove from the component
   * \param response true on the success field if the node unload was succefully, otherwise false
   *   and the error_message field contains the error.
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void
  OnUnloadNode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response);

  /// Service callback to get the list of nodes in the component
  /*
   * Return a two list: one with the unique identifiers and other with full name of the nodes.
   *
   * \param request_header unused
   * \param request unused
   * \param response list with the unique ids and full node names
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void
  OnListNodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response);

private:
  std::weak_ptr<rclcpp::Executor> executor_;

  uint64_t unique_id_ {1};
  std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders_;
  std::map<uint64_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;

  rclcpp::Service<LoadNode>::SharedPtr loadNode_srv_;
  rclcpp::Service<UnloadNode>::SharedPtr unloadNode_srv_;
  rclcpp::Service<ListNodes>::SharedPtr listNodes_srv_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__
