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
  return subscriptions.empty() &&
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

template<typename EntityCollectionType>
void check_ready(
  EntityCollectionType & collection,
  std::deque<rclcpp::AnyExecutable> & executables,
  size_t size_of_waited_entities,
  typename EntityCollectionType::Key * waited_entities,
  std::function<bool(rclcpp::AnyExecutable &,
  typename EntityCollectionType::EntitySharedPtr &)> fill_executable)
{
  for (size_t ii = 0; ii < size_of_waited_entities; ++ii) {
    if (waited_entities[ii]) {
      auto entity_iter = collection.find(waited_entities[ii]);
      if (entity_iter != collection.end()) {
        auto entity = entity_iter->second.entity.lock();
        if (!entity) {
          continue;
        }

        auto callback_group = entity_iter->second.callback_group.lock();
        if (callback_group && !callback_group->can_be_taken_from().load()) {
          continue;
        }
        rclcpp::AnyExecutable exec;

        exec.callback_group = callback_group;
        if (fill_executable(exec, entity)) {
          executables.push_back(exec);
        }
      }
    }
  }
}

std::deque<rclcpp::AnyExecutable>
ready_executables(
  const ExecutorEntitiesCollection & collection,
  rclcpp::WaitResult<rclcpp::WaitSet> & wait_result
)
{
  std::deque<rclcpp::AnyExecutable> ret;

  if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
    return ret;
  }
  auto rcl_wait_set = wait_result.get_wait_set().get_rcl_wait_set();
  check_ready(
    collection.timers,
    ret,
    rcl_wait_set.size_of_timers,
    rcl_wait_set.timers,
    [](rclcpp::AnyExecutable & exec, auto timer) {
      if (!timer->call()) {
        return false;
      }
      exec.timer = timer;
      return true;
    });

  check_ready(
    collection.subscriptions,
    ret,
    rcl_wait_set.size_of_subscriptions,
    rcl_wait_set.subscriptions,
    [](rclcpp::AnyExecutable & exec, auto subscription) {
      exec.subscription = subscription;
      return true;
    });


  check_ready(
    collection.services,
    ret,
    rcl_wait_set.size_of_services,
    rcl_wait_set.services,
    [](rclcpp::AnyExecutable & exec, auto service) {
      exec.service = service;
      return true;
    });

  check_ready(
    collection.clients,
    ret,
    rcl_wait_set.size_of_clients,
    rcl_wait_set.clients,
    [](rclcpp::AnyExecutable & exec, auto client) {
      exec.client = client;
      return true;
    });

  for (auto & [handle, entry] : collection.waitables) {
    auto waitable = entry.entity.lock();
    if (waitable && waitable->is_ready(&rcl_wait_set)) {
      auto group = entry.callback_group.lock();
      if (group && !group->can_be_taken_from().load()) {
        continue;
      }

      rclcpp::AnyExecutable exec;
      exec.waitable = waitable;
      exec.callback_group = group;
      ret.push_back(exec);
    }
  }
  return ret;
}

}  // namespace executors
}  // namespace rclcpp
