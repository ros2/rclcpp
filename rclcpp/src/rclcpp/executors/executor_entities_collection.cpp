// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors/executor_entities_collection.hpp"

namespace rclcpp
{
namespace executors
{
bool ExecutorEntitiesCollection::empty() const
{
  return
    subscriptions.empty() &&
    timers.empty() &&
    guard_conditions.empty() &&
    clients.empty() &&
    services.empty() &&
    waitables.empty();
}

void ExecutorEntitiesCollection::clear()
{
  subscriptions.clear();
  timers.clear();
  guard_conditions.clear();
  clients.clear();
  services.clear();
  waitables.clear();
}

size_t ExecutorEntitiesCollection::remove_expired_entities()
{
  auto remove_entities = [](auto & collection) -> size_t {
      size_t removed = 0;
      for (auto it = collection.begin(); it != collection.end(); ) {
        if (it->second.entity.expired()) {
          ++removed;
          it = collection.erase(it);
        } else {
          ++it;
        }
      }
      return removed;
    };

  return
    remove_entities(subscriptions) +
    remove_entities(timers) +
    remove_entities(guard_conditions) +
    remove_entities(clients) +
    remove_entities(services) +
    remove_entities(waitables);
}

void
build_entities_collection(
  const std::vector<rclcpp::CallbackGroup::WeakPtr> & callback_groups,
  ExecutorEntitiesCollection & collection)
{
  collection.clear();

  for (auto weak_group_ptr : callback_groups) {
    auto group_ptr = weak_group_ptr.lock();
    if (!group_ptr) {
      continue;
    }

    if (group_ptr->can_be_taken_from().load()) {
      group_ptr->collect_all_ptrs(
        [&collection, weak_group_ptr](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
          collection.subscriptions.insert(
            {
              subscription->get_subscription_handle().get(),
              {subscription, weak_group_ptr}
            });
        },
        [&collection, weak_group_ptr](const rclcpp::ServiceBase::SharedPtr & service) {
          collection.services.insert(
            {
              service->get_service_handle().get(),
              {service, weak_group_ptr}
            });
        },
        [&collection, weak_group_ptr](const rclcpp::ClientBase::SharedPtr & client) {
          collection.clients.insert(
            {
              client->get_client_handle().get(),
              {client, weak_group_ptr}
            });
        },
        [&collection, weak_group_ptr](const rclcpp::TimerBase::SharedPtr & timer) {
          collection.timers.insert(
            {
              timer->get_timer_handle().get(),
              {timer, weak_group_ptr}
            });
        },
        [&collection, weak_group_ptr](const rclcpp::Waitable::SharedPtr & waitable) {
          collection.waitables.insert(
            {
              waitable.get(),
              {waitable, weak_group_ptr}
            });
        }
      );
    }
  }
}

size_t
ready_executables(
  const ExecutorEntitiesCollection & collection,
  rclcpp::WaitResult<rclcpp::WaitSet> & wait_result,
  std::deque<rclcpp::AnyExecutable> & executables
)
{
  size_t added = 0;
  if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
    return added;
  }
  auto rcl_wait_set = wait_result.get_wait_set().get_rcl_wait_set();

  // Cache shared pointers to groups to avoid extra work re-locking them
  std::map<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::CallbackGroup::SharedPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>> group_map;

  auto group_cache = [&group_map](const rclcpp::CallbackGroup::WeakPtr & weak_cbg_ptr)
    {
      if (group_map.count(weak_cbg_ptr) == 0) {
        group_map.insert({weak_cbg_ptr, weak_cbg_ptr.lock()});
      }
      return group_map.find(weak_cbg_ptr)->second;
    };

  for (size_t ii = 0; ii < rcl_wait_set.size_of_timers; ++ii) {
    if (nullptr == rcl_wait_set.timers[ii]) {continue;}
    auto entity_iter = collection.timers.find(rcl_wait_set.timers[ii]);
    if (entity_iter != collection.timers.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      auto group_info = group_cache(entity_iter->second.callback_group);
      if (!group_info || !group_info->can_be_taken_from().load()) {
        continue;
      }
      if (!entity->call()) {
        continue;
      }
      rclcpp::AnyExecutable exec;
      exec.timer = entity;
      exec.callback_group = group_info;
      executables.push_back(exec);
      added++;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_subscriptions; ++ii) {
    if (nullptr == rcl_wait_set.subscriptions[ii]) {continue;}
    auto entity_iter = collection.subscriptions.find(rcl_wait_set.subscriptions[ii]);
    if (entity_iter != collection.subscriptions.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      auto group_info = group_cache(entity_iter->second.callback_group);
      if (!group_info || !group_info->can_be_taken_from().load()) {
        continue;
      }
      rclcpp::AnyExecutable exec;
      exec.subscription = entity;
      exec.callback_group = group_info;
      executables.push_back(exec);
      added++;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_services; ++ii) {
    if (nullptr == rcl_wait_set.services[ii]) {continue;}
    auto entity_iter = collection.services.find(rcl_wait_set.services[ii]);
    if (entity_iter != collection.services.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      auto group_info = group_cache(entity_iter->second.callback_group);
      if (!group_info || !group_info->can_be_taken_from().load()) {
        continue;
      }
      rclcpp::AnyExecutable exec;
      exec.service = entity;
      exec.callback_group = group_info;
      executables.push_back(exec);
      added++;
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_clients; ++ii) {
    if (nullptr == rcl_wait_set.clients[ii]) {continue;}
    auto entity_iter = collection.clients.find(rcl_wait_set.clients[ii]);
    if (entity_iter != collection.clients.end()) {
      auto entity = entity_iter->second.entity.lock();
      if (!entity) {
        continue;
      }
      auto group_info = group_cache(entity_iter->second.callback_group);
      if (!group_info || !group_info->can_be_taken_from().load()) {
        continue;
      }
      rclcpp::AnyExecutable exec;
      exec.client = entity;
      exec.callback_group = group_info;
      executables.push_back(exec);
      added++;
    }
  }

  for (const auto & [handle, entry] : collection.waitables) {
    auto waitable = entry.entity.lock();
    if (!waitable) {
      continue;
    }
    if (!waitable->is_ready(rcl_wait_set)) {
      continue;
    }
    auto group_info = group_cache(entry.callback_group);
    if (!group_info || !group_info->can_be_taken_from().load()) {
      continue;
    }
    rclcpp::AnyExecutable exec;
    exec.waitable = waitable;
    exec.callback_group = group_info;
    executables.push_back(exec);
    added++;
  }
  return added;
}
}  // namespace executors
}  // namespace rclcpp
