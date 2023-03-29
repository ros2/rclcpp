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

#ifndef RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTION_HPP_
#define RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTION_HPP_

#include <deque>
#include <unordered_map>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"

#include <rclcpp/any_executable.hpp>
#include <rclcpp/node_interfaces/node_base.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/executor_notify_waitable.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rclcpp/wait_result.hpp>
#include <rclcpp/wait_set.hpp>

namespace rclcpp
{
namespace executors
{

struct ExecutorEntitiesCollection
{
  struct TimerEntry
  {
    rclcpp::TimerBase::WeakPtr timer;
    rclcpp::CallbackGroup::WeakPtr callback_group;
  };
  using TimerCollection = std::unordered_map<const rcl_timer_t *, TimerEntry>;

  struct SubscriptionEntry
  {
    rclcpp::SubscriptionBase::WeakPtr subscription;
    rclcpp::CallbackGroup::WeakPtr callback_group;
  };
  using SubscriptionCollection = std::unordered_map<const rcl_subscription_t *, SubscriptionEntry>;

  struct ClientEntry
  {
    rclcpp::ClientBase::WeakPtr client;
    rclcpp::CallbackGroup::WeakPtr callback_group;
  };
  using ClientCollection = std::unordered_map<const rcl_client_t *, ClientEntry>;

  struct ServiceEntry
  {
    rclcpp::ServiceBase::WeakPtr service;
    rclcpp::CallbackGroup::WeakPtr callback_group;
  };
  using ServiceCollection = std::unordered_map<const rcl_service_t *, ServiceEntry>;

  struct WaitableEntry
  {
    rclcpp::Waitable::WeakPtr waitable;
    rclcpp::CallbackGroup::WeakPtr callback_group;
  };
  using WaitableCollection = std::unordered_map<const rclcpp::Waitable *, WaitableEntry>;

  using GuardConditionCollection = std::unordered_map<const rcl_guard_condition_t *,
      rclcpp::GuardCondition::WeakPtr>;

  TimerCollection timers;
  SubscriptionCollection subscriptions;
  ClientCollection clients;
  ServiceCollection services;
  GuardConditionCollection guard_conditions;
  WaitableCollection waitables;

  void clear();

  using TimerUpdatedFunc = std::function<void (rclcpp::TimerBase::SharedPtr)>;
  void update_timers(
    const ExecutorEntitiesCollection::TimerCollection & other,
    TimerUpdatedFunc timer_added,
    TimerUpdatedFunc timer_removed);

  using SubscriptionUpdatedFunc = std::function<void (rclcpp::SubscriptionBase::SharedPtr)>;
  void update_subscriptions(
    const ExecutorEntitiesCollection::SubscriptionCollection & other,
    SubscriptionUpdatedFunc subscription_added,
    SubscriptionUpdatedFunc subscription_removed);

  using ClientUpdatedFunc = std::function<void (rclcpp::ClientBase::SharedPtr)>;
  void update_clients(
    const ExecutorEntitiesCollection::ClientCollection & other,
    ClientUpdatedFunc client_added,
    ClientUpdatedFunc client_removed);

  using ServiceUpdatedFunc = std::function<void (rclcpp::ServiceBase::SharedPtr)>;
  void update_services(
    const ExecutorEntitiesCollection::ServiceCollection & other,
    ServiceUpdatedFunc service_added,
    ServiceUpdatedFunc service_removed);

  using GuardConditionUpdatedFunc = std::function<void (rclcpp::GuardCondition::SharedPtr)>;
  void update_guard_conditions(
    const ExecutorEntitiesCollection::GuardConditionCollection & other,
    GuardConditionUpdatedFunc guard_condition_added,
    GuardConditionUpdatedFunc guard_condition_removed);

  using WaitableUpdatedFunc = std::function<void (rclcpp::Waitable::SharedPtr)>;
  void update_waitables(
    const ExecutorEntitiesCollection::WaitableCollection & other,
    WaitableUpdatedFunc waitable_added,
    WaitableUpdatedFunc waitable_removed);
};

void
build_entities_collection(
  const std::vector<rclcpp::CallbackGroup::WeakPtr> & callback_groups,
  ExecutorEntitiesCollection & collection);

std::deque<rclcpp::AnyExecutable>
ready_executables(
  const ExecutorEntitiesCollection & collection,
  rclcpp::WaitResult<rclcpp::WaitSet> & wait_result
);

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTION_HPP_
