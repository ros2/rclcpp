// Copyright 2024 Cellumation GmbH.
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
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <deque>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/executors/callback_group_state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

template<class Executable>
class ExecutionGroup
{
public:
  ExecutionGroup(size_t expected_size = 0)
  {
    ready_executables.reserve(expected_size);
  }

  void clear_and_prepare(size_t expected_size)
  {
    ready_executables.clear();
    ready_executables.reserve(expected_size);
    next_unprocessed_ready_executable = 0;
  }

  void add_ready_executable(const Executable & e)
  {
    ready_executables.push_back(e);
  }

  bool has_unprocessed_executables()
  {
    for (; next_unprocessed_ready_executable < ready_executables.size();
      next_unprocessed_ready_executable++)
    {
      const auto & ready_executable = ready_executables[next_unprocessed_ready_executable];

      if (ready_executable.lock()) {
        return true;
      }
    }
    return false;
  }

  bool get_unprocessed_executable(AnyExecutable & any_executable)
  {
    for (; next_unprocessed_ready_executable < ready_executables.size();
      next_unprocessed_ready_executable++)
    {
      const auto & ready_executable = ready_executables[next_unprocessed_ready_executable];

      if (fill_any_executable(any_executable, ready_executable)) {
        // mark the current element as processed
        next_unprocessed_ready_executable++;

        return true;
      }
    }

    return false;
  }

private:
  bool fill_any_executable(
    AnyExecutable & any_executable,
    const rclcpp::SubscriptionBase::WeakPtr & ptr)
  {
    any_executable.subscription = ptr.lock();

    if (any_executable.subscription) {
      //RCUTILS_LOG_INFO("Executing subscription");
    }

    return any_executable.subscription.operator bool();
  }
  bool fill_any_executable(AnyExecutable & any_executable, const rclcpp::TimerBase::WeakPtr & ptr)
  {
    any_executable.timer = ptr.lock();
    if (any_executable.timer) {
        auto data = any_executable.timer->call();
        if (!data) {
          // timer was cancelled, skip it.
          return false;
        }

        any_executable.data = *data;

        return true;
      //RCUTILS_LOG_INFO("Executing timer");
    }
    return false;
  }
  bool fill_any_executable(AnyExecutable & any_executable, const rclcpp::ServiceBase::WeakPtr & ptr)
  {
    any_executable.service = ptr.lock();
    if (any_executable.service) {
      //RCUTILS_LOG_INFO("Executing service");
    }
    return any_executable.service.operator bool();
  }
  bool fill_any_executable(AnyExecutable & any_executable, const rclcpp::ClientBase::WeakPtr & ptr)
  {
    any_executable.client = ptr.lock();
    if (any_executable.client) {
      //RCUTILS_LOG_INFO("Executing client");
    }
    return any_executable.client.operator bool();
  }
  bool fill_any_executable(AnyExecutable & any_executable, const rclcpp::Waitable::WeakPtr & ptr)
  {
    any_executable.waitable = ptr.lock();
    if (any_executable.waitable) {
        any_executable.data = any_executable.waitable->take_data();
        return true;
    }

    return false;
  }

  std::vector<Executable> ready_executables;
  size_t next_unprocessed_ready_executable = 0;

};

class CallbackGroupScheduler
{
public:

  enum SchedulingPolicy
  {
    // Execute all ready events in priority order
    PrioritizedFistInFirstOut,
    // Only execute the highest ready event
    Prioritized,
  };

  CallbackGroupScheduler(SchedulingPolicy sched_policy = SchedulingPolicy::PrioritizedFistInFirstOut) :
    sched_policy(sched_policy)
  {
  }

  void clear_and_prepare(const CallbackGroupState & cb_elements);

  void add_ready_executable(const rclcpp::SubscriptionBase::WeakPtr & executable);
  void add_ready_executable(const rclcpp::ServiceBase::WeakPtr & executable);
  void add_ready_executable(const rclcpp::TimerBase::WeakPtr & executable);
  void add_ready_executable(const rclcpp::ClientBase::WeakPtr & executable);
  void add_ready_executable(const rclcpp::Waitable::WeakPtr & executable);

  enum Priorities
  {
    Timer = 0,
    Subscription,
    Service,
    Client,
    Waitable
  };

  bool get_unprocessed_executable(AnyExecutable & any_executable, enum Priorities for_priority);

  bool has_unprocessed_executables();

private:
  ExecutionGroup<rclcpp::TimerBase::WeakPtr> ready_timers;
  ExecutionGroup<rclcpp::SubscriptionBase::WeakPtr> ready_subscriptions;
  ExecutionGroup<rclcpp::ServiceBase::WeakPtr> ready_services;
  ExecutionGroup<rclcpp::ClientBase::WeakPtr> ready_clients;
  ExecutionGroup<rclcpp::Waitable::WeakPtr> ready_waitables;

  SchedulingPolicy sched_policy;
};

struct AnyExecutableWeakRefCache;
struct RCLToRCLCPPMap;

class CBGExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CBGExecutor)

  /**
   * For the yield_before_execute option, when true std::this_thread::yield()
   * will be called after acquiring work (as an AnyExecutable) and
   * releasing the spinning lock, but before executing the work.
   * This is useful for reproducing some bugs related to taking work more than
   * once.
   *
   * \param options common options for all executors
   * \param number_of_threads number of threads to have in the thread pool,
   *   the default 0 will use the number of cpu cores found (minimum of 2)
   * \param yield_before_execute if true std::this_thread::yield() is called
   * \param timeout maximum time to wait
   */
  RCLCPP_PUBLIC
  explicit CBGExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  virtual ~CBGExecutor();

  RCLCPP_PUBLIC
  virtual void
  add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify = true);

  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups();

  RCLCPP_PUBLIC
  virtual void
  remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    bool notify = true);

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


  // add a callback group to the executor, not bound to any node
  void add_callback_group_only(rclcpp::CallbackGroup::SharedPtr group_ptr);

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin();

  virtual void
  spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  virtual void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

  /**
   * @return true if work was available and executed
   */
  bool collect_and_execute_ready_events(
    std::chrono::nanoseconds max_duration,
    bool recollect_if_no_work_available);

  virtual void
  spin_all(std::chrono::nanoseconds max_duration);

  RCLCPP_PUBLIC
  void
  execute_any_executable(AnyExecutable & any_exec);


  /// Cancel any running spin* function, causing it to return.
  /**
   * This function can be called asynchonously from any thread.
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  void
  cancel();

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads();

  bool
  is_spinning()
  {
    return spinning;
  }

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
      spin_once_internal(timeout_left);

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

protected:
  RCLCPP_PUBLIC
  void
  run(size_t this_thread_number);

  void wait_for_work(std::chrono::nanoseconds timeout, bool do_not_wait_if_all_groups_busy = true);

  struct CallbackGroupData
  {
    CallbackGroup::WeakPtr callback_group;

    std::unique_ptr<CallbackGroupScheduler> scheduler;
    std::unique_ptr<CallbackGroupState> callback_group_state;
    std::unique_ptr<AnyExecutableWeakRefCache> executable_cache;

    bool callback_group_state_needs_update = false;
  };

  bool
  get_next_ready_executable(AnyExecutable & any_executable);


  void fill_callback_group_data(
    rcl_wait_set_s & wait_set,
    const std::vector<CallbackGroupData *> idle_callback_groups,
    const RCLToRCLCPPMap & mapping);

private:
  void sync_callback_groups();

  void spin_once_internal(std::chrono::nanoseconds timeout);

  RCLCPP_DISABLE_COPY(CBGExecutor)

  std::deque<std::function<void(void)>> update_functions;

  std::mutex added_callback_groups_mutex_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> added_callback_groups;

  std::mutex added_nodes_mutex_;
  std::vector<node_interfaces::NodeBaseInterface::WeakPtr> added_nodes;

  //FIXME make unique_ptr
  std::vector<CallbackGroupData> callback_groups;

  std::mutex wait_mutex_;
  size_t number_of_threads_;

  std::chrono::nanoseconds next_exec_timeout_;

  std::atomic_bool needs_callback_group_resync;

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  std::shared_ptr<rclcpp::GuardCondition> interrupt_guard_condition_;

  /// Guard condition for signaling the rmw layer to wake up for system shutdown.
  std::shared_ptr<rclcpp::GuardCondition> shutdown_guard_condition_;

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  /// shutdown callback handle registered to Context
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

  /// Stores the executables for the internal guard conditions
  /// e.g. interrupt_guard_condition_ and shutdown_guard_condition_
  std::unique_ptr<AnyExecutableWeakRefCache> global_executable_cache;

  /// Stores the executables for guard conditions of the nodes
  std::unique_ptr<AnyExecutableWeakRefCache> nodes_executable_cache;


};

}  // namespace executors
}  // namespace rclcpp
