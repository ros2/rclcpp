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
template<typename ACTION>
typename Client<ACTION>::SharedPtr
create_client(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node->get_node_waitables_interface();
  std::weak_ptr<rclcpp::callback_group::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](Client<ACTION> * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (!shared_node) {
        return;
      }
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<Client<ACTION>> fake_shared_ptr(ptr, [](Client<ACTION> *) {});

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

  std::shared_ptr<Client<ACTION>> action_client(
    new Client<ACTION>(node->get_node_base_interface(), node->get_node_logging_interface(), name),
    deleter);

  node->get_node_waitables_interface()->add_waitable(action_client, group);
  return action_client;
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CREATE_CLIENT_HPP_
