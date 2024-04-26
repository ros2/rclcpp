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
#include <mutex>
#include <string>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/executors/executor_notify_waitable.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/executor_entities_collection.hpp"
#include "rclcpp/executors/executor_entities_collector.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"

namespace rclcpp
{

// Forward declaration is used in convenience method signature.
class Node;
class ExecutorImplementation;

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
  virtual void
  spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  spin_node_some(std::shared_ptr<rclcpp::Node> node);

  /// Collect work once and execute all available work, optionally within a max duration.
  /**
   * This function can be overridden.
   * The default implementation is suitable for a single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking or long running
   * callbacks may cause the function exceed the max_duration significantly.
   *
   * If there is no work to be done when this called, it will return immediately
   * because the collecting of available work is non-blocking.
   * Before each piece of ready work is executed this function checks if the
   * max_duration has been exceeded, and if it has it returns without starting
   * the execution of the next piece of work.
   *
   * If a max_duration of 0 is given, then all of the collected work will be
   * executed before the function returns.
   *
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   */
  RCLCPP_PUBLIC
  virtual void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

  /// Add a node, complete all immediately available work exhaustively, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   */
  RCLCPP_PUBLIC
  virtual void
  spin_node_all(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds max_duration);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  spin_node_all(std::shared_ptr<rclcpp::Node> node, std::chrono::nanoseconds max_duration);

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


  /// Collect work once and execute the next available work, optionally within a duration.
  /**
   * This function can be overridden.  The default implementation is suitable for
   * a single-thread model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   * \param[in] timeout The maximum amount of time to spend waiting for work.
   *   `-1` is potentially block forever waiting for work.
   */
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
    return spin_until_future_complete_impl(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout),
      [&future](std::chrono::nanoseconds wait_time) {
        return future.wait_for(wait_time);
      }
    );
  }

  /// Cancel any running spin* function, causing it to return.
  /**
   * This function can be called asynchonously from any thread.
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  virtual void
  cancel();

  /// Returns true if the executor is currently spinning.
  /**
   * This function can be called asynchronously from any thread.
   * \return True if the executor is currently spinning.
   */
  RCLCPP_PUBLIC
  bool
  is_spinning();

protected:
  /// Constructor that will not initialize any non-trivial members.
  /**
  * This constructor is intended to be used by any derived executor
  * that explicitly does not want to use the default implementation provided
  * by this class.
  */
  explicit Executor(const std::shared_ptr<rclcpp::Context> & context);

  /// Add a node to executor, execute the next available unit of work, and remove the node.
  /**
   * Implementation of spin_node_once using std::chrono::nanoseconds
   * \param[in] node Shared pointer to the node to add.
   * \param[in] timeout How long to wait for work to become available. Negative values cause
   *   spin_node_once to block indefinitely (the default behavior). A timeout of 0 causes this
   *   function to be non-blocking.
   */
  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds timeout);

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \sa spin_until_future_complete()
   * The only difference with spin_until_future_complete() is that the future's
   * type is obscured through a std::function which lets you wait on it
   * reguardless of type.
   *
   * \param[in] timeout see spin_until_future_complete() for details
   * \param[in] wait_for_future function to wait on the future and get the
   *   status after waiting
   */
  RCLCPP_PUBLIC
  virtual FutureReturnCode
  spin_until_future_complete_impl(
    std::chrono::nanoseconds timeout,
    const std::function<std::future_status(std::chrono::nanoseconds wait_time)> & wait_for_future);

  /// Collect work and execute available work, optionally within a duration.
  /**
   * Implementation of spin_some and spin_all.
   * The exhaustive flag controls if the function will re-collect available work within the duration.
   *
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   * \param[in] exhaustive when set to true, continue to collect work and execute (spin_all)
   *                       when set to false, return when all collected work is executed (spin_some)
   */
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

  /// Run subscription executable.
  /**
   * Do necessary setup and tear-down as well as executing the subscription.
   * \param[in] subscription Subscription to execute
   */
  RCLCPP_PUBLIC
  static void
  execute_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription);

  /// Run timer executable.
  /**
   * Do necessary setup and tear-down as well as executing the timer callback.
   * \param[in] timer Timer to execute
   */
  RCLCPP_PUBLIC
  static void
  execute_timer(rclcpp::TimerBase::SharedPtr timer, const std::shared_ptr<void> & data_ptr);

  /// Run service server executable.
  /**
   * Do necessary setup and tear-down as well as executing the service server callback.
   * \param[in] service Service to execute
   */
  RCLCPP_PUBLIC
  static void
  execute_service(rclcpp::ServiceBase::SharedPtr service);

  /// Run service client executable.
  /**
   * Do necessary setup and tear-down as well as executing the service client callback.
   * \param[in] service Service to execute
   */
  RCLCPP_PUBLIC
  static void
  execute_client(rclcpp::ClientBase::SharedPtr client);

  /// Gather all of the waitable entities from associated nodes and callback groups.
  RCLCPP_PUBLIC
  void
  collect_entities();

  /// Block until more work becomes avilable or timeout is reached.
  /**
   * Builds a set of waitable entities, which are passed to the middleware.
   * After building wait set, waits on middleware to notify.
   * \param[in] timeout duration to wait for new work to become available.
   * \throws std::runtime_error if the wait set can be cleared
   */
  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Check for executable in ready state and populate union structure.
  /**
   * \param[out] any_executable populated union structure of ready executable
   * \return true if an executable was ready and any_executable was populated,
   *   otherwise false
   */
  RCLCPP_PUBLIC
  bool
  get_next_ready_executable(AnyExecutable & any_executable);

  /// Wait for executable in ready state and populate union structure.
  /**
   * If an executable is ready, it will return immediately, otherwise
   * block based on the timeout for work to become ready.
   *
   * \param[out] any_executable populated union structure of ready executable
   * \param[in] timeout duration of time to wait for work, a negative value
   *   (the defualt behavior), will make this function block indefinitely
   * \return true if an executable was ready and any_executable was populated,
   *   otherwise false
   */
  RCLCPP_PUBLIC
  bool
  get_next_executable(
    AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// This function triggers a recollect of all entities that are registered to the executor.
  /**
   * Calling this function is thread safe.
   *
   * \param[in] notify if true will execute a trigger that will wake up a waiting executor
   */
  void
  trigger_entity_recollect(bool notify);

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  std::shared_ptr<rclcpp::GuardCondition> interrupt_guard_condition_;

  /// Guard condition for signaling the rmw layer to wake up for system shutdown.
  std::shared_ptr<rclcpp::GuardCondition> shutdown_guard_condition_;

  mutable std::mutex mutex_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

  RCLCPP_DISABLE_COPY(Executor)

  RCLCPP_PUBLIC
  virtual void
  spin_once_impl(std::chrono::nanoseconds timeout);

  /// Waitable containing guard conditions controlling the executor flow.
  /**
   * This waitable contains the interrupt and shutdown guard condition, as well
   * as the guard condition associated with each node and callback group.
   * By default, if any change is detected in the monitored entities, the notify
   * waitable will awake the executor and rebuild the collections.
   */
  std::shared_ptr<rclcpp::executors::ExecutorNotifyWaitable> notify_waitable_;

  std::atomic_bool entities_need_rebuild_;

  /// Collector used to associate executable entities from nodes and guard conditions
  rclcpp::executors::ExecutorEntitiesCollector collector_;

  /// WaitSet to be waited on.
  rclcpp::WaitSet wait_set_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
  std::optional<rclcpp::WaitResult<rclcpp::WaitSet>> wait_result_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// Hold the current state of the collection being waited on by the waitset
  rclcpp::executors::ExecutorEntitiesCollection current_collection_ RCPPUTILS_TSA_GUARDED_BY(
    mutex_);

  /// Hold the current state of the notify waitable being waited on by the waitset
  std::shared_ptr<rclcpp::executors::ExecutorNotifyWaitable> current_notify_waitable_
  RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// shutdown callback handle registered to Context
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  /// Pointer to implementation
  std::unique_ptr<ExecutorImplementation> impl_;
};

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_
