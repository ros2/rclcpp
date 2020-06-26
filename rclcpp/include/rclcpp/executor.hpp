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
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/scope_exit.hpp"

namespace rclcpp
{

// Forward declaration is used in convenience method signature.
class Node;

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
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Executor)

  /// Default constructor.
  /**
   * \param[in] options Options used to configure the executor.
   */
  RCLCPP_PUBLIC
  explicit Executor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

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
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::add_node
   */
  RCLCPP_PUBLIC
  virtual void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Remove a node from the executor.
  /**
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   */
  RCLCPP_PUBLIC
  virtual void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  virtual void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Add a node to executor, execute the next available unit of work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   * \param[in] timeout How long to wait for work to become available. Negative values cause
   * spin_node_once to block indefinitely (the default behavior). A timeout of 0 causes this
   * function to be non-blocking.
   */
  template<typename RepT = int64_t, typename T = std::milli>
  void
  spin_node_once(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1))
  {
    return spin_node_once_nanoseconds(
      node,
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  template<typename NodeT = rclcpp::Node, typename RepT = int64_t, typename T = std::milli>
  void
  spin_node_once(
    std::shared_ptr<NodeT> node,
    std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1))
  {
    return spin_node_once_nanoseconds(
      node->get_node_base_interface(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  /// Add a node, complete all immediately available work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   */
  RCLCPP_PUBLIC
  void
  spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  void
  spin_node_some(std::shared_ptr<rclcpp::Node> node);

  /// Collect work once and execute all available work, optionally within a duration.
  /**
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   *
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   * Note that spin_some() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */
  RCLCPP_PUBLIC
  virtual void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

  /// Collect and execute work repeatedly within a duration or until no more work is available.
  /**
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   * If the time that waitables take to be executed is longer than the period on which new waitables
   * become ready, this method will execute work repeatedly until `max_duration` has elapsed.
   *
   * \param[in] max_duration The maximum amount of time to spend executing work. Must be positive.
   * Note that spin_all() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */
  RCLCPP_PUBLIC
  virtual void
  spin_all(std::chrono::nanoseconds max_duration);

  RCLCPP_PUBLIC
  virtual void
  spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \param[in] future The future to wait on. If this function returns SUCCESS, the future can be
   *   accessed without blocking (though it may still throw an exception).
   * \param[in] timeout Optional timeout parameter, which gets passed to Executor::spin_node_once.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   */
  template<typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  FutureReturnCode
  spin_until_future_complete(
    const FutureT & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
    // inside a callback executed by an executor.

    // Check the future before entering the while loop.
    // If the future is already complete, don't try to spin.
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    if (spinning.exchange(true)) {
      throw std::runtime_error("spin_until_future_complete() called while already spinning");
    }
    RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
    while (rclcpp::ok(this->context_) && spinning.load()) {
      // Do one item of work.
      spin_once_impl(timeout_left);

      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return FutureReturnCode::INTERRUPTED;
  }

  /// Cancel any running spin* function, causing it to return.
  /**
   * This function can be called asynchonously from any thread.
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  void
  cancel();

  /// Support dynamic switching of the memory strategy.
  /**
   * Switching the memory strategy while the executor is spinning in another threading could have
   * unintended consequences.
   * \param[in] memory_strategy Shared pointer to the memory strategy to set.
   * \throws std::runtime_error if memory_strategy is null
   */
  RCLCPP_PUBLIC
  void
  set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

protected:
  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  void
  spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

  /// Find the next available executable and do the work associated with it.
  /**
   * \param[in] any_exec Union structure that can hold any executable type (timer, subscription,
   * service, client).
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  void
  execute_any_executable(AnyExecutable & any_exec);

  RCLCPP_PUBLIC
  static void
  execute_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  static void
  execute_service(rclcpp::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  static void
  execute_client(rclcpp::ClientBase::SharedPtr client);

  /**
   * \throws std::runtime_error if the wait set can be cleared
   */
  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_by_group(rclcpp::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  bool
  get_next_ready_executable(AnyExecutable & any_executable);

  RCLCPP_PUBLIC
  bool
  get_next_executable(
    AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rcl_guard_condition_t interrupt_guard_condition_ = rcl_get_zero_initialized_guard_condition();

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  // Mutex to protect the subsequent memory_strategy_.
  std::mutex memory_strategy_mutex_;

  /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

  RCLCPP_DISABLE_COPY(Executor)

  RCLCPP_PUBLIC
  void
  spin_once_impl(std::chrono::nanoseconds timeout);

  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
  std::list<const rcl_guard_condition_t *> guard_conditions_;
};

namespace executor
{

using Executor [[deprecated("use rclcpp::Executor instead")]] = rclcpp::Executor;

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_
