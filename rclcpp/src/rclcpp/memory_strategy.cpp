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
#include <memory>

using rclcpp::memory_strategy::MemoryStrategy;

rclcpp::SubscriptionBase::SharedPtr
MemoryStrategy::get_subscription_by_handle(
  std::shared_ptr<const rcl_subscription_t> subscriber_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    auto match_subscription = group->find_subscription_ptrs_if(
      [&subscriber_handle](const rclcpp::SubscriptionBase::SharedPtr & subscription) -> bool {
        return subscription->get_subscription_handle() == subscriber_handle;
      });
    if (match_subscription) {
      return match_subscription;
    }
  }
  return nullptr;
}

rclcpp::ServiceBase::SharedPtr
MemoryStrategy::get_service_by_handle(
  std::shared_ptr<const rcl_service_t> service_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    auto service_ref = group->find_service_ptrs_if(
      [&service_handle](const rclcpp::ServiceBase::SharedPtr & service) -> bool {
        return service->get_service_handle() == service_handle;
      });
    if (service_ref) {
      return service_ref;
    }
  }
  return nullptr;
}

rclcpp::ClientBase::SharedPtr
MemoryStrategy::get_client_by_handle(
  std::shared_ptr<const rcl_client_t> client_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    auto client_ref = group->find_client_ptrs_if(
      [&client_handle](const rclcpp::ClientBase::SharedPtr & client) -> bool {
        return client->get_client_handle() == client_handle;
      });
    if (client_ref) {
      return client_ref;
    }
  }
  return nullptr;
}

rclcpp::TimerBase::SharedPtr
MemoryStrategy::get_timer_by_handle(
  std::shared_ptr<const rcl_timer_t> timer_handle,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    if (!group) {
      continue;
    }
    auto timer_ref = group->find_timer_ptrs_if(
      [&timer_handle](const rclcpp::TimerBase::SharedPtr & timer) -> bool {
        return timer->get_timer_handle() == timer_handle;
      });
    if (timer_ref) {
      return timer_ref;
    }
  }
  return nullptr;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MemoryStrategy::get_node_by_group(
  rclcpp::CallbackGroup::SharedPtr group,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  if (!group) {
    return nullptr;
  }

  rclcpp::CallbackGroup::WeakPtr weak_group_ptr(group);
  const auto finder = weak_groups_to_nodes.find(weak_group_ptr);
  if (finder != weak_groups_to_nodes.end()) {
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr = finder->second.lock();
    return node_ptr;
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!group || !node) {
      continue;
    }
    auto match_sub = group->find_subscription_ptrs_if(
      [&subscription](const rclcpp::SubscriptionBase::SharedPtr & sub) -> bool {
        return sub == subscription;
      });
    if (match_sub) {
      return group;
    }
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_service(
  rclcpp::ServiceBase::SharedPtr service,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!group || !node) {
      continue;
    }
    auto service_ref = group->find_service_ptrs_if(
      [&service](const rclcpp::ServiceBase::SharedPtr & serv) -> bool {
        return serv == service;
      });
    if (service_ref) {
      return group;
    }
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_client(
  rclcpp::ClientBase::SharedPtr client,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!group || !node) {
      continue;
    }
    auto client_ref = group->find_client_ptrs_if(
      [&client](const rclcpp::ClientBase::SharedPtr & cli) -> bool {
        return cli == client;
      });
    if (client_ref) {
      return group;
    }
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_timer(
  rclcpp::TimerBase::SharedPtr timer,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!group || !node) {
      continue;
    }
    auto timer_ref = group->find_timer_ptrs_if(
      [&timer](const rclcpp::TimerBase::SharedPtr & time) -> bool {
        return time == timer;
      });
    if (timer_ref) {
      return group;
    }
  }
  return nullptr;
}

rclcpp::CallbackGroup::SharedPtr
MemoryStrategy::get_group_by_waitable(
  rclcpp::Waitable::SharedPtr waitable,
  const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
{
  for (const auto & pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();
    if (!group || !node) {
      continue;
    }
    auto waitable_ref = group->find_waitable_ptrs_if(
      [&waitable](const rclcpp::Waitable::SharedPtr & group_waitable) -> bool {
        return group_waitable == waitable;
      });
    if (waitable_ref) {
      return group;
    }
  }
  return nullptr;
}
