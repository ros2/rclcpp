// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__CREATE_CLIENT_HPP_
#define RCLCPP_ACTION__CREATE_CLIENT_HPP_

#include <rclcpp/node.hpp>

#include <memory>
#include <string>

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
/// Create an action client.
/**
 * This function is equivalent to \sa create_client()` however is using the individual
 * node interfaces to create the client.
 *
 * \param[in] node_base_interface The node base interface of the corresponding node.
 * \param[in] node_graph_interface The node graph interface of the corresponding node.
 * \param[in] node_logging_interface The node logging interface of the corresponding node.
 * \param[in] node_waitables_interface The node waitables interface of the corresponding node.
 * \param[in] name The action name.
 * \param[in] group The action client will be added to this callback group.
 *   If `nullptr`, then the action client is added to the default callback group.
 */
template<typename ActionT>
typename Client<ActionT>::SharedPtr
create_client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  const std::string & name,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node_waitables_interface;
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](Client<ActionT> * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (!shared_node) {
        return;
      }
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<Client<ActionT>> fake_shared_ptr(ptr, [](Client<ActionT> *) {});

      if (group_is_null) {
        // Was added to default group
        shared_node->remove_waitable(fake_shared_ptr, nullptr);
      } else {
        // Was added to a specfic group
        auto shared_group = weak_group.lock();
        if (shared_group) {
          shared_node->remove_waitable(fake_shared_ptr, shared_group);
        }
      }
      delete ptr;
    };

  std::shared_ptr<Client<ActionT>> action_client(
    new Client<ActionT>(
      node_base_interface,
      node_graph_interface,
      node_logging_interface,
      name),
    deleter);

  node_waitables_interface->add_waitable(action_client, group);
  return action_client;
}

/// Create an action client.
/**
 * \param[in] node The action client will be added to this node.
 * \param[in] name The action name.
 * \param[in] group The action client will be added to this callback group.
 *   If `nullptr`, then the action client is added to the default callback group.
 */
template<typename ActionT, typename NodeT>
typename Client<ActionT>::SharedPtr
create_client(
  NodeT node,
  const std::string & name,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return rclcpp_action::create_client<ActionT>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    name,
    group);
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CREATE_CLIENT_HPP_
