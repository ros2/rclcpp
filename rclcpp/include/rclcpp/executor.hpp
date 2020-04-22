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

#include "rclcpp/callback_group.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executor_policies/timer_favoring_priority_queue.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"

namespace rclcpp
{

/// Base class for Executor providing the common interface for adding items, spinning, etc.
class ExecutorBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ExecutorBase)

  /// Default constructor.
  /**
   * \param[in] options Options for the executor.
   */
  RCLCPP_PUBLIC
  explicit ExecutorBase(const ExecutorOptions & options = ExecutorOptions());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~ExecutorBase();

  /// Execution loop which waits for work, executes work, and repeats until canceled.
  /**
   * This will block, continuing to wait for work and then execute it, until
   * canceled, either by the cancel() method or by the associated context being
   * shutdown, either explicitly or due to a SIGINT (perhaps due to ctrl-c).
   */
  virtual
  void
  spin() = 0;

  /// Add all of a node's callback groups to the executor.
  /**
   * Add all of the callback groups of a node to this executor.
   *
   * If any callback groups are associated with another executor, this method
   * will throw a std::runtime_error.
   *
   * It will also trigger the interrupt guard condition which will cause the
   * executor to wake up and consider the changes, then go back to waiting.
   * Unless the notify parameter is passed false, in which case it will not
   * interrupt the executor, and the changes may not be considered immediately.
   *
   * \param[in] node_ptr Shared pointer to the node which will have callback groups added.
   * \param[in] notify If true, notfiy the executor of changes, otherwise do not.
   * \throws std::runtime_error if any callback groups are associated with another executor.
   */
  template<class NodeT>
  void
  add_node(const std::shared_ptr<NodeT> & node_ptr, bool notify = true)
  {
    this->add_node(
      node_ptr->get_node_base_interface(),
      notify,
      false  // raise on encountering already associated callback groups
    );
  }

  /// Overload that takes the NodeBaseInterface directly.
  template<class NodeT>
  void
  add_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
    bool notify = true)
  {
    this->add_node(
      node_ptr,
      notify,
      false  // raise on encountering already associated callback groups
    );
  }

  /// Add all unassociated callback groups of the given node to this executor.
  /**
   * Same as add_node(), but instead of throwing if a callback group is already
   * associated with another exector (already added to it) it will just ignore
   * it rather than throwing.
   *
   * \param[in] node_ptr Shared pointer to the node which will have callback groups added.
   * \param[in] notify If true, notfiy the executor of changes, otherwise do not.
   */
  template<class NodeT>
  void
  add_unassociated_callback_groups_from_node(
    const std::shared_ptr<NodeT> & node_ptr,
    bool notify = true)
  {
    this->add_node(
      node_ptr->get_node_base_interface(),
      notify,
      true  // ignore already associated callback groups
    );
  }

  /// Overload that takes the NodeBaseInterface directly.
  template<class NodeT>
  void
  add_unassociated_callback_groups_from_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
    bool notify = true)
  {
    this->add_node(
      node_ptr,
      notify,
      true  // ignore already associated callback groups
    );
  }

  /// Remove all of a node's callback groups from the executor.
  /**
   * Remove all of the callback groups of a node from this executor.
   *
   * It will also trigger the interrupt guard condition which will cause the
   * executor to wake up and consider the changes, then go back to waiting.
   * Unless the notify parameter is passed false, in which case it will not
   * interrupt the executor, and the changes may not be considered immediately.
   *
   * \param[in] node Node which will have callback groups removed.
   * \param[in] notify If true, notfiy the executor of changes, otherwise do not.
   */
  template<class NodeT>
  void
  remove_node(const NodeT & node, bool notify = true)
  {
    this->remove_node(*node.get_node_base_interface(), notify);
  }

  /// Overload that takes a shared pointer to the node.
  /**
   * This is kept for backwards compatibility from when executors shared
   * ownership of Nodes.
   */
  template<class NodeT>
  void
  remove_node(const std::shared_ptr<NodeT> & node_ptr, bool notify = true)
  {
    this->remove_node(*node_ptr->get_node_base_interface(), notify);
  }

  /// Placeholder used to indicate that a method overload should not notify the executor.
  struct DoNotNotify {};

  /// Add a callback group to this executor.
  /**
   * If the given callback group is already associated with another executor,
   * this method will throw a std::runtime_error.
   *
   * This overload of add_callback_group() will notify the executor so it will
   * wake up if waiting and consider the changes.
   *
   * Weak ownership of the callback group is kept by the executor all of the
   * time, but while waiting the weak ownership is periodically elevated to
   * shared ownership.
   * Therefore, if you let the callback group shared pointer go out of scope
   * then it will stay in scope until this executor is done using it, at which
   * point the callback group will be destructed and automatically removed from
   * this executor in the next pass.
   *
   * \param[in] callback_group_ptr The callback group to be added.
   * \throws std::runtime_error if the callback group is associated with another
   *   executor already.
   * \throws std::invalid_argument if the callback group pointer is nullptr.
   */
  RCLCPP_PUBLIC
  virtual
  void
  add_callback_group(rclcpp::CallbackGroup::SharedPtr callback_group_ptr) = 0;

  /// Add a callback group to this executor without notifying the executor.
  /**
   * The same as the other overload of add_callback_group(), except it does not
   * notify the executor, so it will not wake up and these changes may not be
   * considered immediately.
   *
   * Note, a bool with a default value would be preferable for controlling the
   * notify behavior, and we're using it in the add/remove node above, but
   * in order to keep this function virtual, and to avoid using default values
   * in conjunction with virtual methods, we use an overload instead, in the
   * spirit of std::nothrow_t, e.g.:
   *   https://en.cppreference.com/w/cpp/memory/new/nothrow
   */
  RCLCPP_PUBLIC
  virtual
  void
  add_callback_group(rclcpp::CallbackGroup::SharedPtr callback_group_ptr, DoNotNotify) = 0;

  /// Remove a callback group from this executor.
  /**
   * If the given callback group is not associated with this executor, this
   * method will throw a std::runtime_error.
   *
   * This overload of add_callback_group() will notify the executor so it will
   * wake up if waiting and consider the changes.
   *
   * \param[in] callback_group The callback group to be removed.
   * \throws std::runtime_error if the callback group is not associated with
   *   this executor.
   * \throws std::invalid_argument if the callback group pointer is nullptr.
   */
  RCLCPP_PUBLIC
  virtual
  void
  remove_callback_group(const rclcpp::CallbackGroup & callback_group) = 0;

  /// Remove a callback group from this executor without notifying the executor.
  /**
   * The same as the other overload of remove_callback_group(), except it does not
   * notify the executor, so it will not wake up and these changes may not be
   * considered immediately.
   *
   * See add_callback_group() for a note about the use of DoNotNotify.
   */
  RCLCPP_PUBLIC
  virtual
  void
  remove_callback_group(const rclcpp::CallbackGroup & callback_group, DoNotNotify) = 0;

  /// Add a node to executor, execute the next available unit of work, and remove the node.
  /**
   * \param[in] node Shared pointer to the node to add.
   * \param[in] timeout How long to wait for work to become available.
   *   Negative values cause spin_node_once to block indefinitely (the default
   *   behavior).
   *   A timeout of 0 causes this function to be non-blocking.
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
  template<typename NodeT, typename RepT = int64_t, typename T = std::milli>
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
   * \param[in] node Shared pointer to the node to spin some.
   */
  template<class NodeT>
  void
  spin_node_some(const std::shared_ptr<NodeT> & node)
  {
    this->spin_node_some(node->get_node_base_interface());
  }

  /// Complete all available queued work without blocking.
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
  virtual
  void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) = 0;

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
  template<typename ResponseT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  FutureReturnCode
  spin_until_future_complete(
    const std::shared_future<ResponseT> & future,
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

    while (rclcpp::ok(this->context_)) {
      // Do one item of work.
      spin_once(timeout_left);
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
   * This function can be called asynchronously from any thread.
   */
  RCLCPP_PUBLIC
  virtual
  void
  cancel() = 0;

protected:
  /// Implementation of add_node().
  /**
   * \param[in] node_ptr The node which will have its callback groups added.
   * \param[in] notify If true, the executor is interrupted to consider the
   *   changes, otherwise it is not interrupted.
   * \param[in] ignore_associated_callback_groups If true, then when a callback
   *   group which is already been added to another executor is encountered
   *   it will be ignored, if false then std::runtime_error is thrown instead.
   * \throws std::runtime_error if ignore_associated_callback_groups is false
   *   and a callback group which is already associated with another executor
   *   is encountered.
   * \throws std::invalid_argument if node_ptr is nullptr.
   */
  RCLCPP_PUBLIC
  virtual
  void
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify,
    bool ignore_associated_callback_groups) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify) = 0;

  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  virtual
  void
  spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

  /// Find the next available executable and do the work associated with it.
  /** \param[in] any_exec Union structure that can hold any executable type (timer, subscription,
   * service, client).
   */
  RCLCPP_PUBLIC
  void
  execute_any_executable(rclcpp::AnyExecutable & any_exec);

  RCLCPP_PUBLIC
  static
  void
  execute_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static
  void
  execute_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  static
  void
  execute_service(rclcpp::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  static
  void
  execute_client(rclcpp::ClientBase::SharedPtr client);

  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  bool
  get_next_ready_executable(rclcpp::AnyExecutable & any_executable);

  RCLCPP_PUBLIC
  bool
  get_next_executable(
    rclcpp::AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_DISABLE_COPY(ExecutorBase)

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rclcpp::GuardCondition interrupt_guard_condition_;
  // rcl_guard_condition_t interrupt_guard_condition_ = rcl_get_zero_initialized_guard_condition();

  // /// Wait set for managing entities that the rmw layer waits on.
  // rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  // // Mutex to protect the subsequent memory_strategy_.
  // std::mutex memory_strategy_mutex_;

  // /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  // memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  /// The context associated with this executor.
  rclcpp::Context::SharedPtr context_;

  // std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
  // std::list<const rcl_guard_condition_t *> guard_conditions_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> weak_guard_conditions_;

};

/// Template class which serves as the foundation of actual executors.
/**
 * This class combines the wait set, scheduling policy, and the ExecutorBase
 * class, and implements all of the pure virtual functions of ExecutorBase
 * making it a concrete class.
 */
template<class WaitSetT, class SchedulingPolicy>
class ExecutorTemplate : public ExecutorBase, public SchedulingPolicy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ExecutorTemplate)

  /// Default constructor.
  /**
   * \param[in] options Options for the executor.
   */
  explicit ExecutorTemplate(const ExecutorOptions & options = ExecutorOptions())
  : ExecutorBase(options), SchedulingPolicy(options), WaitSetT(options.context)
  {}

  /// Default virtual destructor.
  RCLCPP_PUBLIC
  virtual
  ~ExecutorTemplate() = default;

protected:
  WaitSetT wait_set_;
};

/// Executor concept which waits for work and coordinates execution of user callbacks.
/**
 * Executor provides spin functions (including spin_node_once and spin_some).
 * It coordinates the nodes and callback groups by looking for available work
 * and completing it, based on the threading or concurrency scheme provided by
 * the subclass implementation.
 * An example of available work is executing a subscription callback, or a
 * timer callback.
 * The executor structure allows for a decoupling of the communication graph
 * and the execution model.
 * See SingleThreadedExecutor and MultiThreadedExecutor for examples of
 * different execution paradigms.
 *
 * By default this alias provides a foundation based on specific wait set type
 * and a scheduling policy.
 * The wait set is expected to be dynamic, i.e. items can be added or removed
 * after creation, and thread-safe, i.e. items can be added or removed while
 * also waiting concurrently.
 */
using Executor = ExecutorTemplate<
  rclcpp::ThreadSafeWaitSet,
  rclcpp::executor_policies::TimerFavoringPriorityQueue>;

namespace executor
{

using Executor [[deprecated("use rclcpp::Executor instead")]] = Executor;

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_
