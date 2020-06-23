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

#ifndef RCLCPP_ACTION__CREATE_SERVER_HPP_
#define RCLCPP_ACTION__CREATE_SERVER_HPP_

#include <rcl_action/action_server.h>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_waitables_interface.hpp>

#include <memory>
#include <string>

#include "rclcpp_action/server.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
/// Create an action server.
/**
 * All provided callback functions must be non-blocking.
 * This function is equivalent to \sa create_server()` however is using the individual
 * node interfaces to create the server.
 *
 * \sa Server::Server() for more information.
 *
 * \param[in] node_base_interface The node base interface of the corresponding node.
 * \param[in] node_clock_interface The node clock interface of the corresponding node.
 * \param[in] node_logging_interface The node logging interface of the corresponding node.
 * \param[in] node_waitables_interface The node waitables interface of the corresponding node.
 * \param[in] name The action name.
 * \param[in] handle_goal A callback that decides if a goal should be accepted or rejected.
 * \param[in] handle_cancel A callback that decides if a goal should be attempted to be canceled.
 *  The return from this callback only indicates if the server will try to cancel a goal.
 *  It does not indicate if the goal was actually canceled.
 * \param[in] handle_accepted A callback that is called to give the user a handle to the goal.
 * \param[in] options options to pass to the underlying `rcl_action_server_t`.
 * \param[in] group The action server will be added to this callback group.
 *   If `nullptr`, then the action server is added to the default callback group.
 */
template<typename ActionT>
typename Server<ActionT>::SharedPtr
create_server(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  const std::string & name,
  typename Server<ActionT>::GoalCallback handle_goal,
  typename Server<ActionT>::CancelCallback handle_cancel,
  typename Server<ActionT>::AcceptedCallback handle_accepted,
  const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node_waitables_interface;
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](Server<ActionT> * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (!shared_node) {
        return;
      }
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<Server<ActionT>> fake_shared_ptr(ptr, [](Server<ActionT> *) {});

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

  std::shared_ptr<Server<ActionT>> action_server(new Server<ActionT>(
      node_base_interface,
      node_clock_interface,
      node_logging_interface,
      name,
      options,
      handle_goal,
      handle_cancel,
      handle_accepted), deleter);

  node_waitables_interface->add_waitable(action_server, group);
  return action_server;
}

/// Create an action server.
/**
 * All provided callback functions must be non-blocking.
 *
 * \sa Server::Server() for more information.
 *
 * \param[in] node] The action server will be added to this node.
 * \param[in] name The action name.
 * \param[in] handle_goal A callback that decides if a goal should be accepted or rejected.
 * \param[in] handle_cancel A callback that decides if a goal should be attempted to be canceled.
 *  The return from this callback only indicates if the server will try to cancel a goal.
 *  It does not indicate if the goal was actually canceled.
 * \param[in] handle_accepted A callback that is called to give the user a handle to the goal.
 * \param[in] options options to pass to the underlying `rcl_action_server_t`.
 * \param[in] group The action server will be added to this callback group.
 *   If `nullptr`, then the action server is added to the default callback group.
 */
template<typename ActionT, typename NodeT>
typename Server<ActionT>::SharedPtr
create_server(
  NodeT node,
  const std::string & name,
  typename Server<ActionT>::GoalCallback handle_goal,
  typename Server<ActionT>::CancelCallback handle_cancel,
  typename Server<ActionT>::AcceptedCallback handle_accepted,
  const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_server<ActionT>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    name,
    handle_goal,
    handle_cancel,
    handle_accepted,
    options,
    group);
}
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CREATE_SERVER_HPP_
