// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <vector>

#include "rcutils/executor_event_types.h"

#include "rclcpp/executor.hpp"
#include "rclcpp/executors/events_executor_entities_collector.hpp"
#include "rclcpp/executors/events_executor_notify_waitable.hpp"
#include "rclcpp/executors/timers_manager.hpp"
#include "rclcpp/node.hpp"

namespace rclcpp
{
namespace executors
{

/// Events executor implementation
/**
 * Add description
 *
 * To run this executor:
 * rclcpp::executors::EventsExecutor executor;
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
   * \param[in] options Options used to configure the executor.
   */
  RCLCPP_PUBLIC
  explicit EventsExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  /// Default destrcutor.
  RCLCPP_PUBLIC
  virtual ~EventsExecutor() = default;

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
   * executor.provide_callbacks();
   * while(condition) {
   *   executor.spin_some();
   * }
   */
  RCLCPP_PUBLIC
  void
  spin_some(std::chrono::nanoseconds max_duration) override;

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

  // Executor callback: Push new events into the queue and trigger cv.
  // This function is called by the DDS entities when an event happened,
  // like a subscription receiving a message.
  static void
  push_event(const void * executor_ptr, ExecutorEvent event)
  {
    // Cast executor_ptr to this (need to remove constness)
    auto this_executor = const_cast<executors::EventsExecutor *>(
      static_cast<const executors::EventsExecutor *>(executor_ptr));

    // Event queue mutex scope
    {
      std::unique_lock<std::mutex> lock(this_executor->push_mutex_);

      this_executor->event_queue_.push_back(event);
    }
    // Notify that the event queue has some events in it.
    this_executor->event_queue_cv_.notify_one();
  }

  // This function allows to remove an entity from the EventsExecutor.
  // Entities are any of SubscriptionBase, PublisherBase, ClientBase, ServerBase, Waitable.
  // After an entity has been removed using this API, it can be safely destroyed without the risk
  // that the executor would try to access it again.
  template<typename T>
  void
  remove_entity(T * entity)
  {
    // We need to unset the callbacks to make sure that after removing events from the
    // queues, this entity will not push anymore before being completely destroyed.
    // This assumes that all supported entities implement this function.
    entity->set_events_executor_callback(nullptr, nullptr);

    // Remove events associated with this entity from the event queue
    {
      std::unique_lock<std::mutex> lock(push_mutex_);
      event_queue_.erase(
        std::remove_if(
          event_queue_.begin(), event_queue_.end(),
          [&entity](ExecutorEvent event) {return event.entity == entity;}), event_queue_.end());
    }

    // Remove events associated with this entity from the local event queue
    {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      local_event_queue_.erase(
        std::remove_if(
          local_event_queue_.begin(), local_event_queue_.end(),
          [&entity](ExecutorEvent event) {
            return event.entity == entity;
          }), local_event_queue_.end());
    }
  }

  /// Add a callback group to an executor.
  /**
   * \sa rclcpp::Executor::add_callback_group
   */
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
  RCLCPP_PUBLIC
  void
  spin_once_impl(std::chrono::nanoseconds timeout) override;

private:
  RCLCPP_DISABLE_COPY(EventsExecutor)

  // Event queue implementation is a deque only to
  // facilitate the removal of events from expired entities.
  using EventQueue = std::deque<ExecutorEvent>;

  /// Extract and execute events from the queue until it is empty
  RCLCPP_PUBLIC
  void
  consume_all_events(EventQueue & queue);

  // Execute a single event
  RCLCPP_PUBLIC
  void
  execute_event(const ExecutorEvent & event);

  // We use two instances of EventQueue to allow threads to push events while we execute them
  EventQueue event_queue_;
  EventQueue local_event_queue_;

  EventsExecutorEntitiesCollector::SharedPtr entities_collector_;
  EventsExecutorNotifyWaitable::SharedPtr executor_notifier_;
  // Mutex to protect the insertion of events in the queue
  std::mutex push_mutex_;
  // Mutex to protect the execution of events
  std::mutex execution_mutex_;
  // Variable used to notify when an event is added to the queue
  std::condition_variable event_queue_cv_;
  // Timers manager
  std::shared_ptr<TimersManager> timers_manager_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_HPP_
