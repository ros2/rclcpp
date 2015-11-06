// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTOR_HPP_
#define RCLCPP__EXECUTOR_HPP_

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

#include "rclcpp/any_executable.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executor
{

/// Coordinate the order and timing of available communication tasks.
/**
 * Executor provides spin functions (including spin_node_once and spin_some).
 * It coordinates the nodes and callback groups by looking for available work and completing it,
 * based on the threading or concurrency scheme provided by the subclass implementation.
 * An example of available work is executing a subscription callback, or a timer callback.
 * The executor structure allows for a decoupling of the communication graph and the execution
 * model.
 * See SingleThreadedExecutor and MultiThreadedExecutor for examples of execution paradigms.
 */
class Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Executor);

  /// Default constructor.
  // \param[in] ms The memory strategy to be used with this executor.
  RCLCPP_PUBLIC
  explicit Executor(
    memory_strategy::MemoryStrategy::SharedPtr ms = memory_strategies::create_default_strategy());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~Executor();

  /// Do work periodically as it becomes available to us. Blocking call, may block indefinitely.
  // It is up to the implementation of Executor to implement spin.
  virtual void
  spin() = 0;

  /// Add a node to the executor.
  /**
   * An executor can have zero or more nodes which provide work during `spin` functions.
   * \param[in] node_ptr Shared pointer to the node to be added.
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
   * node was added, it will wake up.
   */
  RCLCPP_PUBLIC
  virtual void
  add_node(rclcpp::node::Node::SharedPtr node_ptr, bool notify = true);

  /// Remove a node from the executor.
  /**
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   */
  RCLCPP_PUBLIC
  virtual void
  remove_node(rclcpp::node::Node::SharedPtr node_ptr, bool notify = true);

  /// Add a node to executor, execute the next available unit of work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   * \param[in] timeout How long to wait for work to become available. Negative values cause
   * spin_node_once to block indefinitely (the default behavior). A timeout of 0 causes this
   * function to be non-blocking.
   */
  template<typename T = std::milli>
  void
  spin_node_once(rclcpp::node::Node::SharedPtr node,
    std::chrono::duration<int64_t, T> timeout = std::chrono::duration<int64_t, T>(-1))
  {
    return spin_node_once_nanoseconds(
      node,
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  /// Add a node, complete all immediately available work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   */
  RCLCPP_PUBLIC
  void
  spin_node_some(rclcpp::node::Node::SharedPtr node);

  /// Complete all available queued work without blocking.
  /**
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   */
  RCLCPP_PUBLIC
  virtual void
  spin_some();

  /// Support dynamic switching of the memory strategy.
  /**
   * Switching the memory strategy while the executor is spinning in another threading could have
   * unintended consequences.
   * \param[in] memory_strategy Shared pointer to the memory strategy to set.
   */
  RCLCPP_PUBLIC
  void
  set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

protected:
  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(rclcpp::node::Node::SharedPtr node, std::chrono::nanoseconds timeout);

  /// Find the next available executable and do the work associated with it.
  /** \param[in] any_exec Union structure that can hold any executable type (timer, subscription,
   * service, client).
   */
  RCLCPP_PUBLIC
  void
  execute_any_executable(AnyExecutable::SharedPtr any_exec);

  RCLCPP_PUBLIC
  static void
  execute_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_intra_process_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_timer(rclcpp::timer::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  static void
  execute_service(rclcpp::service::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  static void
  execute_client(rclcpp::client::ClientBase::SharedPtr client);

  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  rclcpp::node::Node::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::timer::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  void
  get_next_timer(AnyExecutable::SharedPtr any_exec);

  RCLCPP_PUBLIC
  std::chrono::nanoseconds
  get_earliest_timer();

  RCLCPP_PUBLIC
  AnyExecutable::SharedPtr
  get_next_ready_executable();

  RCLCPP_PUBLIC
  AnyExecutable::SharedPtr
  get_next_executable(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rmw_guard_condition_t * interrupt_guard_condition_;

  /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

private:
  RCLCPP_DISABLE_COPY(Executor);

  std::vector<std::weak_ptr<rclcpp::node::Node>> weak_nodes_;
  std::array<void *, 2> guard_cond_handles_;
};

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_
