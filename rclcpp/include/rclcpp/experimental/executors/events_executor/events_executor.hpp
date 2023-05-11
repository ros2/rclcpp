// Copyright 2023 iRobot Corporation.
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

#ifndef RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_
#define RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/executors/executor_entities_collection.hpp"
#include "rclcpp/executors/executor_entities_collector.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor_event_types.hpp"
#include "rclcpp/experimental/executors/events_executor/events_queue.hpp"
#include "rclcpp/experimental/executors/events_executor/simple_events_queue.hpp"
#include "rclcpp/experimental/timers_manager.hpp"
#include "rclcpp/node.hpp"

namespace rclcpp
{
namespace experimental
{
namespace executors
{

/// Events executor implementation
/**
 * This executor uses an events queue and a timers manager to execute entities from its
 * associated nodes and callback groups.
 * ROS 2 entities allow to set callback functions that are invoked when the entity is triggered
 * or has work to do. The events-executor sets these callbacks such that they push an
 * event into its queue.
 *
 * This executor tries to reduce as much as possible the amount of maintenance operations.
 * This allows to use customized `EventsQueue` classes to achieve different goals such
 * as very low CPU usage, bounded memory requirement, determinism, etc.
 *
 * The executor uses a weak ownership model and it locks entities only while executing
 * their related events.
 *
 * To run this executor:
 * rclcpp::experimental::executors::EventsExecutor executor;
 * executor.add_node(node);
 * executor.spin();
 * executor.remove_node(node);
 */
class EventsExecutor : public rclcpp::Executor
{
  friend class EventsExecutorEntitiesCollector;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsExecutor)

  /// Default constructor. See the default constructor for Executor.
  /**
   * \param[in] events_queue The queue used to store events.
   * \param[in] execute_timers_separate_thread If true, timers are executed in a separate
   * thread. If false, timers are executed in the same thread as all other entities.
   * \param[in] options Options used to configure the executor.
   */
  RCLCPP_PUBLIC
  explicit EventsExecutor(
    rclcpp::experimental::executors::EventsQueue::UniquePtr events_queue = std::make_unique<
      rclcpp::experimental::executors::SimpleEventsQueue>(),
    bool execute_timers_separate_thread = false,
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~EventsExecutor();

  /// Events executor implementation of spin.
  /**
   * This function will block until work comes in, execute it, and keep blocking.
   * It will only be interrupted by a CTRL-C (managed by the global signal handler).
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin() override;

  /// Events executor implementation of spin some
  /**
   * This non-blocking function will execute the timers and events
   * that were ready when this API was called, until timeout or no
   * more work available. New ready-timers/events arrived while
   * executing work, won't be taken into account here.
   *
   * Example:
   *   while(condition) {
   *     spin_some();
   *     sleep(); // User should have some sync work or
   *              // sleep to avoid a 100% CPU usage
   *   }
   */
  RCLCPP_PUBLIC
  void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  /// Events executor implementation of spin all
  /**
   * This non-blocking function will execute timers and events
   * until timeout or no more work available. If new ready-timers/events
   * arrive while executing work available, they will be executed
   * as long as the timeout hasn't expired.
   *
   * Example:
   *   while(condition) {
   *     spin_all();
   *     sleep(); // User should have some sync work or
   *              // sleep to avoid a 100% CPU usage
   *   }
   */
  RCLCPP_PUBLIC
  void
  spin_all(std::chrono::nanoseconds max_duration) override;

  /// Add a node to the executor.
  /**
   * \sa rclcpp::Executor::add_node
   */
  RCLCPP_PUBLIC
  void
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \sa rclcpp::EventsExecutor::add_node
   */
  RCLCPP_PUBLIC
  void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  /// Remove a node from the executor.
  /**
   * \sa rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  void
  remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \sa rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  /// Add a callback group to an executor.
  /**
   * \sa rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  void
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true) override;

  /// Remove callback group from the executor
  /**
   * \sa rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  void
  remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    bool notify = true) override;

  /// Get callback groups that belong to executor.
  /**
   * \sa rclcpp::Executor::get_all_callback_groups()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups() override;

  /// Get callback groups that belong to executor.
  /**
   * \sa rclcpp::Executor::get_manually_added_callback_groups()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups() override;

  /// Get callback groups that belong to executor.
  /**
   * \sa rclcpp::Executor::get_automatically_added_callback_groups_from_nodes()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes() override;

protected:
  /// Internal implementation of spin_once
  RCLCPP_PUBLIC
  void
  spin_once_impl(std::chrono::nanoseconds timeout) override;

  /// Internal implementation of spin_some
  RCLCPP_PUBLIC
  void
  spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

private:
  RCLCPP_DISABLE_COPY(EventsExecutor)

  /// Execute a provided executor event if its associated entities are available
  void
  execute_event(const ExecutorEvent & event);

  /// Collect entities from callback groups and refresh the current collection with them
  void
  refresh_current_collection_from_callback_groups();

  /// Refresh the current collection using the provided new_collection
  void
  refresh_current_collection(const rclcpp::executors::ExecutorEntitiesCollection & new_collection);

  /// Create a listener callback function for the provided entity
  std::function<void(size_t)>
  create_entity_callback(void * entity_key, ExecutorEventType type);

  /// Create a listener callback function for the provided waitable entity
  std::function<void(size_t, int)>
  create_waitable_callback(const rclcpp::Waitable * waitable_id);

  /// Utility to add the notify waitable to an entities collection
  void
  add_notify_waitable_to_collection(
    rclcpp::executors::ExecutorEntitiesCollection::WaitableCollection & collection);

  /// Searches for the provided entity_id in the collection and returns the entity if valid
  template<typename CollectionType>
  typename CollectionType::EntitySharedPtr
  retrieve_entity(typename CollectionType::Key entity_id, CollectionType & collection)
  {
    // Check if the entity_id is in the collection
    auto it = collection.find(entity_id);
    if (it == collection.end()) {
      return nullptr;
    }

    // Check if the entity associated with the entity_id is valid
    // and remove it from the collection if it isn't
    auto entity = it->second.entity.lock();
    if (!entity) {
      collection.erase(it);
    }

    // Return the retrieved entity (this can be a nullptr if the entity was not valid)
    return entity;
  }

  /// Queue where entities can push events
  rclcpp::experimental::executors::EventsQueue::UniquePtr events_queue_;

  std::shared_ptr<rclcpp::executors::ExecutorEntitiesCollector> entities_collector_;
  std::shared_ptr<rclcpp::executors::ExecutorNotifyWaitable> notify_waitable_;

  /// Mutex to protect the current_entities_collection_
  std::recursive_mutex collection_mutex_;
  std::shared_ptr<rclcpp::executors::ExecutorEntitiesCollection> current_entities_collection_;

  /// Flag used to reduce the number of unnecessary waitable events
  std::atomic<bool> notify_waitable_event_pushed_ {false};

  /// Timers manager used to track and/or execute associated timers
  std::shared_ptr<rclcpp::experimental::TimersManager> timers_manager_;
};

}  // namespace executors
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__EVENTS_EXECUTOR_HPP_
