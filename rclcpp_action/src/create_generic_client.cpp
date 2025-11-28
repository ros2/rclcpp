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

#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/typesupport_helpers.hpp"

#include "rclcpp_action/create_generic_client.hpp"
#include "rclcpp_action/generic_client.hpp"

namespace rclcpp_action
{
typename GenericClient::SharedPtr
create_generic_client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  const std::string & name,
  const std::string & type,
  rclcpp::CallbackGroup::SharedPtr group,
  const rcl_action_client_options_t & options)
{
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node_waitables_interface;
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](GenericClient * ptr)
    {
      if (nullptr == ptr) {
        return;
      }
      auto shared_node = weak_node.lock();
      if (shared_node) {
        // API expects a shared pointer, give it one with a deleter that does nothing.
        std::shared_ptr<GenericClient> fake_shared_ptr(ptr, [](GenericClient *) {});

        if (group_is_null) {
          // Was added to default group
          shared_node->remove_waitable(fake_shared_ptr, nullptr);
        } else {
          // Was added to a specific group
          auto shared_group = weak_group.lock();
          if (shared_group) {
            shared_node->remove_waitable(fake_shared_ptr, shared_group);
          }
        }
      }
      delete ptr;
    };

  auto typesupport_lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  auto action_typesupport_handle = rclcpp::get_action_typesupport_handle(
    type, "rosidl_typesupport_cpp", *typesupport_lib);

  std::shared_ptr<GenericClient> action_client(
    new GenericClient(
      node_base_interface,
      node_graph_interface,
      node_logging_interface,
      name,
      typesupport_lib,
      action_typesupport_handle,
      options),
    deleter);

  node_waitables_interface->add_waitable(action_client, group);
  return action_client;
}

}  // namespace rclcpp_action
