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

void ExecutorEntitiesCollection::clear()
{
  subscriptions.clear();
  timers.clear();
  guard_conditions.clear();
  clients.clear();
  services.clear();
  waitables.clear();
}

void ExecutorEntitiesCollection::update_timers(
  const ExecutorEntitiesCollection::TimerCollection & other,
  TimerUpdatedFunc timer_added,
  TimerUpdatedFunc timer_removed)
{
  for (auto it = this->timers.begin(); it != this->timers.end(); ) {
    if (other.count(it->first) == 0) {
      auto timer = it->second.timer.lock();
      if (timer) {
        timer_removed(timer);
      }
      it = this->timers.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->timers.count(it->first) == 0) {
      auto timer = it->second.timer.lock();
      if (timer) {
        timer_added(timer);
      }
      this->timers.insert(*it);
    }
  }
}

void ExecutorEntitiesCollection::update_subscriptions(
  const ExecutorEntitiesCollection::SubscriptionCollection & other,
  SubscriptionUpdatedFunc subscription_added,
  SubscriptionUpdatedFunc subscription_removed)
{
  for (auto it = this->subscriptions.begin(); it != this->subscriptions.end(); ) {
    if (other.count(it->first) == 0) {
      auto subscription = it->second.subscription.lock();
      if (subscription) {
        subscription_removed(subscription);
      }
      it = this->subscriptions.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->subscriptions.count(it->first) == 0) {
      auto subscription = it->second.subscription.lock();
      if (subscription) {
        subscription_added(subscription);
      }
      this->subscriptions.insert(*it);
    }
  }
}

void ExecutorEntitiesCollection::update_clients(
  const ExecutorEntitiesCollection::ClientCollection & other,
  ClientUpdatedFunc client_added,
  ClientUpdatedFunc client_removed)
{
  for (auto it = this->clients.begin(); it != this->clients.end(); ) {
    if (other.count(it->first) == 0) {
      auto client = it->second.client.lock();
      if (client) {
        client_removed(client);
      }
      it = this->clients.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->clients.count(it->first) == 0) {
      auto client = it->second.client.lock();
      if (client) {
        client_added(client);
      }
      this->clients.insert(*it);
    }
  }
}

void ExecutorEntitiesCollection::update_services(
  const ExecutorEntitiesCollection::ServiceCollection & other,
  ServiceUpdatedFunc service_added,
  ServiceUpdatedFunc service_removed)
{
  for (auto it = this->services.begin(); it != this->services.end(); ) {
    if (other.count(it->first) == 0) {
      auto service = it->second.service.lock();
      if (service) {
        service_removed(service);
      }
      it = this->services.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->services.count(it->first) == 0) {
      auto service = it->second.service.lock();
      if (service) {
        service_added(service);
      }
      this->services.insert(*it);
    }
  }
}

void ExecutorEntitiesCollection::update_guard_conditions(
  const ExecutorEntitiesCollection::GuardConditionCollection & other,
  GuardConditionUpdatedFunc guard_condition_added,
  GuardConditionUpdatedFunc guard_condition_removed)
{
  for (auto it = this->guard_conditions.begin(); it != this->guard_conditions.end(); ) {
    if (other.count(it->first) == 0) {
      auto guard_condition = it->second.lock();
      if (guard_condition) {
        guard_condition_removed(guard_condition);
      }
      it = this->guard_conditions.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->guard_conditions.count(it->first) == 0) {
      auto guard_condition = it->second.lock();
      if (guard_condition) {
        guard_condition_added(guard_condition);
      }
      this->guard_conditions.insert(*it);
    }
  }
}

void ExecutorEntitiesCollection::update_waitables(
  const ExecutorEntitiesCollection::WaitableCollection & other,
  WaitableUpdatedFunc waitable_added,
  WaitableUpdatedFunc waitable_removed)
{
  for (auto it = this->waitables.begin(); it != this->waitables.end(); ) {
    if (other.count(it->first) == 0) {
      auto waitable = it->second.waitable.lock();
      if (waitable) {
        waitable_removed(waitable);
      }
      it = this->waitables.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = other.begin(); it != other.end(); ++it) {
    if (this->waitables.count(it->first) == 0) {
      auto waitable = it->second.waitable.lock();
      if (waitable) {
        waitable_added(waitable);
      }
      this->waitables.insert({it->first, it->second});
    }
  }
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

  for (size_t ii = 0; ii < rcl_wait_set.size_of_timers; ++ii) {
    if (rcl_wait_set.timers[ii]) {
      auto timer_iter = collection.timers.find(rcl_wait_set.timers[ii]);
      if (timer_iter != collection.timers.end()) {
        auto timer = timer_iter->second.timer.lock();
        auto group = timer_iter->second.callback_group.lock();
        if (!timer || !group || !group->can_be_taken_from().load()) {
          continue;
        }

        if (!timer->call()) {
          continue;
        }

        rclcpp::AnyExecutable exec;
        exec.timer = timer;
        exec.callback_group = group;
        ret.push_back(exec);
      }
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_subscriptions; ++ii) {
    if (rcl_wait_set.subscriptions[ii]) {
      auto subscription_iter = collection.subscriptions.find(rcl_wait_set.subscriptions[ii]);
      if (subscription_iter != collection.subscriptions.end()) {
        auto subscription = subscription_iter->second.subscription.lock();
        auto group = subscription_iter->second.callback_group.lock();
        if (!subscription || !group || !group->can_be_taken_from().load()) {
          continue;
        }

        rclcpp::AnyExecutable exec;
        exec.subscription = subscription;
        exec.callback_group = group;
        ret.push_back(exec);
      }
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_services; ++ii) {
    if (rcl_wait_set.services[ii]) {
      auto service_iter = collection.services.find(rcl_wait_set.services[ii]);
      if (service_iter != collection.services.end()) {
        auto service = service_iter->second.service.lock();
        auto group = service_iter->second.callback_group.lock();
        if (!service || !group || !group->can_be_taken_from().load()) {
          continue;
        }

        rclcpp::AnyExecutable exec;
        exec.service = service;
        exec.callback_group = group;
        ret.push_back(exec);
      }
    }
  }

  for (size_t ii = 0; ii < rcl_wait_set.size_of_clients; ++ii) {
    if (rcl_wait_set.clients[ii]) {
      auto client_iter = collection.clients.find(rcl_wait_set.clients[ii]);
      if (client_iter != collection.clients.end()) {
        auto client = client_iter->second.client.lock();
        auto group = client_iter->second.callback_group.lock();
        if (!client || !group || !group->can_be_taken_from().load()) {
          continue;
        }

        rclcpp::AnyExecutable exec;
        exec.client = client;
        exec.callback_group = group;
        ret.push_back(exec);
      }
    }
  }

  for (auto & [handle, entry] : collection.waitables) {
    auto waitable = entry.waitable.lock();
    if (waitable && waitable->is_ready(&rcl_wait_set)) {
      auto group = entry.callback_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
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
