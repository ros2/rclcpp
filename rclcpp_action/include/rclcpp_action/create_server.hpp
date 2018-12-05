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
template<typename ACTION>
typename Server<ACTION>::SharedPtr
create_server(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  typename Server<ACTION>::GoalCallback handle_goal,
  typename Server<ACTION>::CancelCallback handle_cancel,
  typename Server<ACTION>::AcceptedCallback handle_accepted,
  const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
  rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node->get_node_waitables_interface();
  std::weak_ptr<rclcpp::callback_group::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](Server<ACTION> * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (!shared_node) {
        return;
      }
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<Server<ACTION>> fake_shared_ptr(ptr, [](Server<ACTION> *) {});

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

  std::shared_ptr<Server<ACTION>> action_server(new Server<ACTION>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      name,
      options,
      handle_goal,
      handle_cancel,
      handle_accepted), deleter);

  node->get_node_waitables_interface()->add_waitable(action_server, group);
  return action_server;
}
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CREATE_SERVER_HPP_
