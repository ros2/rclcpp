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

template<typename EntityValueType>
struct CollectionEntry
{
  typename EntityValueType::WeakPtr entity;
  rclcpp::CallbackGroup::WeakPtr callback_group;
};

template<typename CollectionType>
void update_entities(
  const CollectionType & update_from,
  CollectionType update_to,
  std::function<void(typename CollectionType::mapped_type::EntitySharedPtr)> on_added,
  std::function<void(typename CollectionType::mapped_type::EntitySharedPtr)> on_removed
)
{
  for (auto it = update_to.begin(); it != update_to.end(); ) {
    if (update_from.count(it->first) == 0) {
      auto entity = it->second.entity.lock();
      if (entity) {
        on_removed(entity);
      }
      it = update_to.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = update_from.begin(); it != update_from.end(); ++it) {
    if (update_to.count(it->first) == 0) {
      auto entity = it->entity.lock();
      if (entity) {
        on_added(entity);
      }
      update_to.insert(*it);
    }
  }
}
template<typename EntityKeyType, typename EntityValueType>
class EntityCollection
  : public std::unordered_map<const EntityKeyType *, CollectionEntry<EntityValueType>>
{
public:
  using Key = const EntityKeyType *;
  using EntityWeakPtr = typename EntityValueType::WeakPtr;
  using EntitySharedPtr = typename EntityValueType::SharedPtr;

  void update(
    const EntityCollection<EntityKeyType, EntityValueType> & other,
    std::function<void(EntitySharedPtr)> on_added,
    std::function<void(EntitySharedPtr)> on_removed)
  {
    update_entities(*this, other, on_added, on_removed);
  }
};

/// Represent the total set of entities for a single executor
/**
 * This allows the entities to be stored from ExecutorEntitiesCollector.
 * The structure also makes in convenient to re-evaluate when entities have been added or removed.
 */
struct ExecutorEntitiesCollection
{
  /// Entity entries for timers
  using TimerCollection = EntityCollection<rcl_timer_t, rclcpp::TimerBase>;

  /// Entity entries for subscriptions
  using SubscriptionCollection = EntityCollection<rcl_subscription_t, rclcpp::SubscriptionBase>;

  /// Entity entries for clients
  using ClientCollection = EntityCollection<rcl_client_t, rclcpp::ClientBase>;

  /// Entity entries for services
  using ServiceCollection = EntityCollection<rcl_service_t, rclcpp::ServiceBase>;

  /// Entity entries for waitables
  using WaitableCollection = EntityCollection<rclcpp::Waitable, rclcpp::Waitable>;

  /// Entity entries for guard conditions
  using GuardConditionCollection = EntityCollection<rcl_guard_condition_t, rclcpp::GuardCondition>;

  TimerCollection timers;
  SubscriptionCollection subscriptions;
  ClientCollection clients;
  ServiceCollection services;
  GuardConditionCollection guard_conditions;
  WaitableCollection waitables;

  void clear();
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
