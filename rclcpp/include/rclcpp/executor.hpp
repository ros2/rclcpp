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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rcpputils/scope_exit.hpp"
#include "rcpputils/mutex.hpp"

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

typedef std::map<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>> WeakCallbackGroupsToNodesMap;

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

  /// Add a callback group to an executor.
  /**
   * An executor can have zero or more callback groups which provide work during `spin` functions.
   * When an executor attempts to add a callback group, the executor checks to see if it is already
   * associated with another executor, and if it has been, then an exception is thrown.
   * Otherwise, the callback group is added to the executor.
   *
   * Adding a callback group with this method does not associate its node with this executor
   * in any way
   *
   * \param[in] group_ptr a shared ptr that points to a callback group
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
   * callback group was added, it will wake up.
   * \throw std::runtime_error if the callback group is associated to an executor
   */
  RCLCPP_PUBLIC
  virtual void
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true);

  /// Get callback groups that belong to executor.
  /**
   * This function returns a vector of weak pointers that point to callback groups that were
   * associated with the executor.
   * The callback groups associated with this executor may have been added with
   * `add_callback_group`, or added when a node was added to the executor with `add_node`, or
   * automatically added when it created by a node already associated with this executor and the
   * automatically_add_to_executor_with_node parameter was true.
   *
   * \return a vector of weak pointers that point to callback groups that are associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups();

  /// Get callback groups that belong to executor.
  /**
   * This function returns a vector of weak pointers that point to callback groups that were
   * associated with the executor.
   * The callback groups associated with this executor have been added with
   * `add_callback_group`.
   *
   * \return a vector of weak pointers that point to callback groups that are associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups();

  /// Get callback groups that belong to executor.
  /**
   * This function returns a vector of weak pointers that point to callback groups that were
   * added from a node that is associated with the executor.
   * The callback groups are added when a node is added to the executor with `add_node`, or
   * automatically if they are created in the future by that node and have the
   * automatically_add_to_executor_with_node argument set to true.
   *
   * \return a vector of weak pointers that point to callback groups from a node associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes();

  /// Remove a callback group from the executor.
  /**
   * The callback group is removed from and disassociated with the executor.
   * If the callback group removed was the last callback group from the node
   * that is associated with the executor, the interrupt guard condition
   * is triggered and node's guard condition is removed from the executor.
   *
   * This function only removes a callback group that was manually added with
   * rclcpp::Executor::add_callback_group.
   * To remove callback groups that were added from a node using
   * rclcpp::Executor::add_node, use rclcpp::Executor::remove_node instead.
   *
   * \param[in] group_ptr Shared pointer to the callback group to be added.
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a
   * callback group was removed, it will wake up.
   * \throw std::runtime_error if node is deleted before callback group
   * \throw std::runtime_error if the callback group is not associated with the executor
   */
  RCLCPP_PUBLIC
  virtual void
  remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    bool notify = true);

  /// Add a node to the executor.
  /**
   * Nodes have associated callback groups, and this method adds any of those callback groups
   * to this executor which have their automatically_add_to_executor_with_node parameter true.
   * The node is also associated with the executor so that future callback groups which are
   * created on the node with the automatically_add_to_executor_with_node parameter set to true
   * are also automatically associated with this executor.
   *
   * Callback groups with the automatically_add_to_executor_with_node parameter set to false must
   * be manually added to an executor using the rclcpp::Executor::add_callback_group method.
   *
   * If a node is already associated with an executor, this method throws an exception.
   *
   * \param[in] node_ptr Shared pointer to the node to be added.
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
   * node was added, it will wake up.
   * \throw std::runtime_error if a node is already associated to an executor
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
   * Any callback groups automatically added when this node was added with
   * rclcpp::Executor::add_node are automatically removed, and the node is no longer associated
   * with this executor.
   *
   * This also means that future callback groups created by the given node are no longer
   * automatically added to this executor.
   *
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   * \throw std::runtime_error if the node is not associated with an executor.
   * \throw std::runtime_error if the node is not associated with this executor.
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
   * \param[in] max_duration The maximum amount of time to spend executing work, must be >= 0.
   *   `0` is potentially block forever until no more work is available.
   * \throw std::invalid_argument if max_duration is less than 0.
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
    RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
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

  /// Returns true if the executor is currently spinning.
  /**
   * This function can be called asynchronously from any thread.
   * \return True if the executor is currently spinning.
   */
  RCLCPP_PUBLIC
  bool
  is_spinning();

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
  get_node_by_group(
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes,
    rclcpp::CallbackGroup::SharedPtr group);

  /// Return true if the node has been added to this executor.
  /**
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \param[in] weak_groups_to_nodes map to nodes to lookup
   * \return true if the node is associated with the executor, otherwise false
   */
  RCLCPP_PUBLIC
  bool
  has_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) const;

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  /// Add a callback group to an executor
  /**
   * \see rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  virtual void
  add_callback_group_to_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    WeakCallbackGroupsToNodesMap & weak_groups_to_nodes,
    bool notify = true) RCPPUTILS_TSA_REQUIRES(mutex_);

  /// Remove a callback group from the executor.
  /**
   * \see rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  virtual void
  remove_callback_group_from_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    WeakCallbackGroupsToNodesMap & weak_groups_to_nodes,
    bool notify = true) RCPPUTILS_TSA_REQUIRES(mutex_);

  RCLCPP_PUBLIC
  bool
  get_next_ready_executable(AnyExecutable & any_executable);

  RCLCPP_PUBLIC
  bool
  get_next_ready_executable_from_map(
    AnyExecutable & any_executable,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes);

  RCLCPP_PUBLIC
  bool
  get_next_executable(
    AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Add all callback groups that can be automatically added from associated nodes.
  /**
   * The executor, before collecting entities, verifies if any callback group from
   * nodes associated with the executor, which is not already associated to an executor,
   * can be automatically added to this executor.
   * This takes care of any callback group that has been added to a node but not explicitly added
   * to the executor.
   * It is important to note that in order for the callback groups to be automatically added to an
   * executor through this function, the node of the callback groups needs to have been added
   * through the `add_node` method.
   */
  RCLCPP_PUBLIC
  virtual void
  add_callback_groups_from_nodes_associated_to_executor() RCPPUTILS_TSA_REQUIRES(mutex_);

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rclcpp::GuardCondition interrupt_guard_condition_;

  std::shared_ptr<rclcpp::GuardCondition> shutdown_guard_condition_;

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  // Mutex to protect the subsequent memory_strategy_.
  mutable rcpputils::PIMutex mutex_;

  /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr
  memory_strategy_ RCPPUTILS_TSA_PT_GUARDED_BY(mutex_);

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

  RCLCPP_DISABLE_COPY(Executor)

  RCLCPP_PUBLIC
  virtual void
  spin_once_impl(std::chrono::nanoseconds timeout);

  typedef std::map<rclcpp::CallbackGroup::WeakPtr,
      const rclcpp::GuardCondition *,
      std::owner_less<rclcpp::CallbackGroup::WeakPtr>>
    WeakCallbackGroupsToGuardConditionsMap;

  /// maps callback groups to guard conditions
  WeakCallbackGroupsToGuardConditionsMap
  weak_groups_to_guard_conditions_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// maps callback groups associated to nodes
  WeakCallbackGroupsToNodesMap
  weak_groups_associated_with_executor_to_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// maps callback groups to nodes associated with executor
  WeakCallbackGroupsToNodesMap
  weak_groups_to_nodes_associated_with_executor_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// maps all callback groups to nodes
  WeakCallbackGroupsToNodesMap
  weak_groups_to_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// nodes that are associated with the executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>
  weak_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// shutdown callback handle registered to Context
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;
};

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_
