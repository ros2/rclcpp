// Copyright 2025 Sony Group Corporation.
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

#ifndef RCLCPP_ACTION__CREATE_GENERIC_CLIENT_HPP_
#define RCLCPP_ACTION__CREATE_GENERIC_CLIENT_HPP_

#include <string>

#include "rclcpp_action/generic_client.hpp"

namespace rclcpp_action
{
/// Create an action generic client.
/**
 * This function is equivalent to \sa create_generic_client()` however is using the individual
 * node interfaces to create the client.
 *
 * \param[in] node_base_interface The node base interface of the corresponding node.
 * \param[in] node_graph_interface The node graph interface of the corresponding node.
 * \param[in] node_logging_interface The node logging interface of the corresponding node.
 * \param[in] node_waitables_interface The node waitables interface of the corresponding node.
 * \param[in] name The action name.
 * \param[in] type The action type.
 * \param[in] group The action client will be added to this callback group.
 *   If `nullptr`, then the action client is added to the default callback group.
 * \param[in] options Options to pass to the underlying `rcl_action_client_t`.
 * \return newly created generic client.
 */
RCLCPP_ACTION_PUBLIC
typename GenericClient::SharedPtr
create_generic_client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  const std::string & name,
  const std::string & type,
  rclcpp::CallbackGroup::SharedPtr group = nullptr,
  const rcl_action_client_options_t & options = rcl_action_client_get_default_options());

/// Create an action generic client.
/**
 * \param[in] node The action client will be added to this node.
 * \param[in] name The action name.
 * \param[in] type The action type.
 * \param[in] group The action client will be added to this callback group.
 *   If `nullptr`, then the action client is added to the default callback group.
 * \param[in] options Options to pass to the underlying `rcl_action_client_t`.
 * \return newly created generic client.
 */
template<typename NodeT>
typename GenericClient::SharedPtr
create_generic_client(
  NodeT node,
  const std::string & name,
  const std::string & type,
  rclcpp::CallbackGroup::SharedPtr group = nullptr,
  const rcl_action_client_options_t & options = rcl_action_client_get_default_options())
{
  return rclcpp_action::create_generic_client(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    name,
    type,
    group,
    options);
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CREATE_GENERIC_CLIENT_HPP_
