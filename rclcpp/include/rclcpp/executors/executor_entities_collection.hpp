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
#include <functional>
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

/// Structure to represent a single entity's entry in a collection
template<typename EntityValueType>
struct CollectionEntry
{
  /// Weak pointer to entity type
  using EntityWeakPtr = typename EntityValueType::WeakPtr;
  /// Shared pointer to entity type
  using EntitySharedPtr = typename EntityValueType::SharedPtr;

  /// The entity
  EntityWeakPtr entity;

  /// If relevant, the entity's corresponding callback_group
  rclcpp::CallbackGroup::WeakPtr callback_group;
};

/// Update a collection based on another collection
/*
 * Iterates update_from and update_to to see which entities have been added/removed between
 * the two collections.
 *
 * For each new entry (in update_from, but not in update_to),
 *   add the entity and fire the on_added callback
 * For each removed entry (in update_to, but not in update_from),
 *   remove the entity and fire the on_removed callback.
 *
 *  \param[in] update_from The collection representing the next iteration's state
 *  \param[inout] update_to The collection representing the current iteration's state
 *  \param[in] on_added Callback fired when a new entity is detected
 *  \param[in] on_removed Callback fired when an entity is removed
 */
template<typename CollectionType>
void update_entities(
  const CollectionType & update_from,
  CollectionType & update_to,
  std::function<void(const typename CollectionType::EntitySharedPtr &)> on_added,
  std::function<void(const typename CollectionType::EntitySharedPtr &)> on_removed
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
      auto entity = it->second.entity.lock();
      if (entity) {
        on_added(entity);
      }
      update_to.insert(*it);
    }
  }
}

/// A collection of entities, indexed by their corresponding handles
template<typename EntityKeyType, typename EntityValueType>
class EntityCollection
  : public std::unordered_map<const EntityKeyType *, CollectionEntry<EntityValueType>>
{
public:
  /// Key type of the map
  using Key = const EntityKeyType *;

  /// Weak pointer to entity type
  using EntityWeakPtr = typename EntityValueType::WeakPtr;

  /// Shared pointer to entity type
  using EntitySharedPtr = typename EntityValueType::SharedPtr;

  /// Update this collection based on the contents of another collection
  /**
   * Update the internal state of this collection, firing callbacks when entities have been
   * added or removed.
   *
   * \param[in] other Collection to compare to
   * \param[in] on_added Callback for when entities have been added
   * \param[in] on_removed Callback for when entities have been removed
   */
  void update(
    const EntityCollection<EntityKeyType, EntityValueType> & other,
    std::function<void(const EntitySharedPtr &)> on_added,
    std::function<void(const EntitySharedPtr &)> on_removed)
  {
    update_entities(other, *this, on_added, on_removed);
  }
};

/// Represent the total set of entities for a single executor
/**
 * This allows the entities to be stored from ExecutorEntitiesCollector.
 * The structure also makes in convenient to re-evaluate when entities have been added or removed.
 */
struct ExecutorEntitiesCollection
{
  /// Collection type for timer entities
  using TimerCollection = EntityCollection<rcl_timer_t, rclcpp::TimerBase>;

  /// Collection type for subscription entities
  using SubscriptionCollection = EntityCollection<rcl_subscription_t, rclcpp::SubscriptionBase>;

  /// Collection type for client entities
  using ClientCollection = EntityCollection<rcl_client_t, rclcpp::ClientBase>;

  /// Collection type for service entities
  using ServiceCollection = EntityCollection<rcl_service_t, rclcpp::ServiceBase>;

  /// Collection type for waitable entities
  using WaitableCollection = EntityCollection<rclcpp::Waitable, rclcpp::Waitable>;

  /// Collection type for guard condition entities
  using GuardConditionCollection = EntityCollection<rcl_guard_condition_t, rclcpp::GuardCondition>;

  /// Collection of timers currently in use by the executor.
  TimerCollection timers;

  /// Collection of subscriptions currently in use by the executor.
  SubscriptionCollection subscriptions;

  /// Collection of clients currently in use by the executor.
  ClientCollection clients;

  /// Collection of services currently in use by the executor.
  ServiceCollection services;

  /// Collection of guard conditions currently in use by the executor.
  GuardConditionCollection guard_conditions;

  /// Collection of waitables currently in use by the executor.
  WaitableCollection waitables;

  /// Check if the entities collection is empty
  /**
   * \return true if all member collections are empty, false otherwise
  */
  bool empty() const;

  /// Clear the entities collection
  void clear();

  /// Remove entities that have expired weak ownership
  /**
   * \return The total number of removed entities
   */
  size_t remove_expired_entities();
};

/// Build an entities collection from callback groups
/**
 * Iterates a list of callback groups and adds entities from each valid group
 *
 * \param[in] callback_groups List of callback groups to check for entities
 * \param[inout] colletion Entities collection to populate with found entities
 */
void
build_entities_collection(
  const std::vector<rclcpp::CallbackGroup::WeakPtr> & callback_groups,
  ExecutorEntitiesCollection & collection);

/// Build a queue of executables ready to be executed
/**
 * Iterates a list of entities and adds them to a queue if they are ready.
 *
 * \param[in] collection Collection of entities corresponding to the current wait set.
 * \param[in] wait_result Result of rclcpp::WaitSet::wait corresponding to the collection.
 * \param[inout] queue of executables to append new ready executables to
 * \return number of new ready executables
 */
size_t
ready_executables(
  const ExecutorEntitiesCollection & collection,
  rclcpp::WaitResult<rclcpp::WaitSet> & wait_result,
  std::deque<rclcpp::AnyExecutable> & executables
);
}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EXECUTOR_ENTITIES_COLLECTION_HPP_
