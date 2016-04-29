// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/memory_strategy.hpp"

using rclcpp::memory_strategy::MemoryStrategy;

rclcpp::subscription::SubscriptionBase::SharedPtr
MemoryStrategy::get_subscription_by_handle(
  const rcl_subscription_t * subscriber_handle, const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_subscription : group->get_subscription_ptrs()) {
        auto subscription = weak_subscription.lock();
        if (subscription) {
          if (subscription->get_subscription_handle() == subscriber_handle) {
            return subscription;
          }
          if (subscription->get_intra_process_subscription_handle() == subscriber_handle) {
            return subscription;
          }
        }
      }
    }
  }
  return nullptr;
}

rclcpp::service::ServiceBase::SharedPtr
MemoryStrategy::get_service_by_handle(const rcl_service_t * service_handle,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & service : group->get_service_ptrs()) {
        if (service->get_service_handle() == service_handle) {
          return service;
        }
      }
    }
  }
  return nullptr;
}

rclcpp::client::ClientBase::SharedPtr
MemoryStrategy::get_client_by_handle(const rcl_client_t * client_handle,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_client : group->get_client_ptrs()) {
        auto client = weak_client.lock();
        if (client && client->get_client_handle() == client_handle) {
          return client;
        }
      }
    }
  }
  return nullptr;
}

/*
rclcpp::parameter_client::ParameterClient::SharedPtr
MemoryStrategy::get_parameter_client_by_handle(
  const WeakNodeVector & weak_nodes)
{
  // iterate through the parameter clients of each node and see if the 
  // need to also indicate which verb was available
}
*/

rclcpp::node::Node::SharedPtr
MemoryStrategy::get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group,
  const WeakNodeVector & weak_nodes)
{
  if (!group) {
    return nullptr;
  }
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto callback_group = weak_group.lock();
      if (callback_group == group) {
        return node;
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_subscription(
  rclcpp::subscription::SubscriptionBase::SharedPtr subscription,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_sub : group->get_subscription_ptrs()) {
        auto sub = weak_sub.lock();
        if (sub == subscription) {
          return group;
        }
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_service(
  rclcpp::service::ServiceBase::SharedPtr service,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & serv : group->get_service_ptrs()) {
        if (serv == service) {
          return group;
        }
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_client(
  rclcpp::client::ClientBase::SharedPtr client,
  const WeakNodeVector & weak_nodes)
{
  for (auto & weak_node : weak_nodes) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      for (auto & weak_client : group->get_client_ptrs()) {
        auto cli = weak_client.lock();
        if (cli && cli == client) {
          return group;
        }
      }
    }
  }
  return nullptr;
}
