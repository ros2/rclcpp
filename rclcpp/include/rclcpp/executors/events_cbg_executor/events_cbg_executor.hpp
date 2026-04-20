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
#include <thread>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

namespace cbg_executor
{
class TimerManager;
struct RegisteredEntityCache;
class CBGScheduler;
struct GlobalWeakExecutableCache;
}

class EventsCBGExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsCBGExecutor)

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
   * \param timeout maximum time to wait
   */
  RCLCPP_PUBLIC
  explicit EventsCBGExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  virtual ~EventsCBGExecutor();

  RCLCPP_PUBLIC
  void
  add_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & group_ptr,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_all_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_manually_added_callback_groups() override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes() override;

  RCLCPP_PUBLIC
  void
  remove_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & group_ptr,
    bool notify = true) override;

  RCLCPP_PUBLIC
  void
  add_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
    bool notify = true) override;

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::add_node
   */
  RCLCPP_PUBLIC
  void
  add_node(const std::shared_ptr<rclcpp::Node> & node_ptr, bool notify = true) override;

  RCLCPP_PUBLIC
  void
  remove_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr,
    bool notify = true) override;

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  void
  remove_node(const std::shared_ptr<rclcpp::Node> & node_ptr, bool notify = true) override;

  // add a callback group to the executor, not bound to any node
  void add_callback_group_only(const rclcpp::CallbackGroup::SharedPtr & group_ptr);

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin() override;

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   * @param exception_handler will be called for every exception in the processing threads
   *
   * The exception_handler can be called from multiple threads at the same time.
   * The exception_handler shall rethrow the exception it if wants to terminate the program.
   */
  RCLCPP_PUBLIC
  void
  spin(const std::function<void(const std::exception &)> & exception_handler);

  RCLCPP_PUBLIC
  void
  spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) override;

  RCLCPP_PUBLIC
  void
  spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  /**
   * @return true if work was available and executed
   */
  RCLCPP_PUBLIC
  bool collect_and_execute_ready_events(
    std::chrono::nanoseconds max_duration,
    bool recollect_if_no_work_available);

  RCLCPP_PUBLIC
  void
  spin_all(std::chrono::nanoseconds max_duration) override;

  /// Cancel any running spin* function, causing it to return.
  /**
   * This function can be called asynchonously from any thread.
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  void
  cancel() override;

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads() const;

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

  /// We need these function to be public, as we use them in the callback_group_scheduler
  using rclcpp::Executor::execute_subscription;
  using rclcpp::Executor::execute_timer;
  using rclcpp::Executor::execute_service;
  using rclcpp::Executor::execute_client;

protected:
  RCLCPP_PUBLIC
  void
  run(size_t this_thread_number, bool block_initially);

  RCLCPP_PUBLIC
  void
  run(
    size_t this_thread_number,
    const std::function<void(const std::exception &)> & exception_handler);

  /**
   * Te be called in termination case. E.g. destructor of shutdown callback.
   * Stops the scheduler and cleans up the internal data structures.
   */
  void shutdown();

  std::unique_ptr<cbg_executor::CBGScheduler> scheduler;

  struct CallbackGroupData
  {
    enum class Origin
    {
      Node,
      ManualAdded,
    };

    CallbackGroup::WeakPtr callback_group;

    std::unique_ptr<cbg_executor::RegisteredEntityCache> registered_entities;

    Origin origin;
  };

  void set_callbacks(CallbackGroupData & cgd);

  /**
   * This function will execute all available executables,
   * that were ready, before this function was called.
   */
  bool execute_previous_ready_executables_until(
    const std::chrono::time_point<std::chrono::steady_clock> & stop_time);

  void unregister_event_callbacks(const rclcpp::CallbackGroup::SharedPtr & cbg) const;

private:
  void remove_all_nodes_and_callback_groups();

  void sync_callback_groups();

  /**
   * Either triggers a sync, or if not spinning,
   * syncs directly.
   */
  void trigger_callback_group_sync();

  RCLCPP_PUBLIC
  void spin_once_internal(std::chrono::nanoseconds timeout);

  RCLCPP_DISABLE_COPY(EventsCBGExecutor)

  std::mutex added_callback_groups_mutex_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> added_callback_groups;

  std::mutex added_nodes_mutex_;
  std::vector<node_interfaces::NodeBaseInterface::WeakPtr> added_nodes;

  std::mutex callback_groups_mutex;

  std::vector<CallbackGroupData> callback_groups;

  size_t number_of_threads_;

  std::chrono::nanoseconds next_exec_timeout_;

  std::atomic_bool needs_callback_group_resync = false;

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// set if we are shutting down.
  bool in_shutdown = false;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  std::shared_ptr<rclcpp::GuardCondition> interrupt_guard_condition_;

  /// Guard condition for signaling the rmw layer to wake up for system shutdown.
  std::shared_ptr<rclcpp::GuardCondition> shutdown_guard_condition_;

  /// shutdown callback handle registered to Context
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

  std::unique_ptr<cbg_executor::TimerManager> timer_manager;

  /// Stores the executables for the internal guard conditions
  /// e.g. interrupt_guard_condition_ and shutdown_guard_condition_
  std::unique_ptr<cbg_executor::GlobalWeakExecutableCache> global_executable_cache;

  /// Stores the executables for guard conditions of the nodes
  std::unique_ptr<cbg_executor::GlobalWeakExecutableCache> nodes_executable_cache;
};

}  // namespace executors
}  // namespace rclcpp
