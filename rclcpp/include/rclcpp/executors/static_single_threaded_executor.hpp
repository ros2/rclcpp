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

#ifndef RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/executable_list.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

/// Static executor implementation
/**
 * This executor is a static version of original single threaded executor.
 * This executor makes the assumption that system does not change during runtime.
 * This means all nodes, callbackgroups, timers, subscriptions etc. are created
 * before .spin() is called.
 *
 * To run this executor instead of SingleThreadedExecutor replace:
 * rclcpp::executors::SingleThreadedExecutor exec;
 * by
 * rclcpp::executors::StaticSingleThreadedExecutor exec;
 * in your source code and spin node(s) in the following way:
 * exec.add_node(node);
 * exec.spin();
 * exec.remove_node(node);
 */
class StaticSingleThreadedExecutor : public executor::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(StaticSingleThreadedExecutor)

  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  StaticSingleThreadedExecutor(
    const executor::ExecutorArgs & args = executor::ExecutorArgs());

  /// Default destrcutor.
  RCLCPP_PUBLIC
  virtual ~StaticSingleThreadedExecutor();

  /// Static executor implementation of spin.
  // This function will block until work comes in, execute it, and keep blocking.
  // It will only be interrupt by a CTRL-C (managed by the global signal handler).
  RCLCPP_PUBLIC
  void
  spin();

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \param[in] future The future to wait on. If this function returns SUCCESS, the future can be
   *   accessed without blocking (though it may still throw an exception).
   * \param[in] timeout Optional timeout parameter, which gets passed to
   *    Executor::execute_ready_executables.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   *
   *  Example usage:
   *  rclcpp::executors::StaticSingleThreadedExecutor exec;
   *  // ... other part of code like creating node
   *  // define future
   *  exec.add_node(node);
   *  exec.spin_until_future_complete(future);
   */
  template<typename ResponseT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  rclcpp::executor::FutureReturnCode
  spin_until_future_complete(
    std::shared_future<ResponseT> & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return rclcpp::executor::FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    rclcpp::executor::ExecutableList executable_list;
    // Collect entities and clean any invalid nodes.
    run_collect_entities();
    get_executable_list(executable_list);
    while (rclcpp::ok(this->context_)) {
      // Do one set of work.
      execute_ready_executables(executable_list, timeout_left);
      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return rclcpp::executor::FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return rclcpp::executor::FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return rclcpp::executor::FutureReturnCode::INTERRUPTED;
  }

protected:
  /// Check which executables in ExecutableList struct are ready from wait_set and execute them.
  /**
   * \param[in] exec_list Structure that can hold subscriptionbases, timerbases, etc
   * \param[in] timeout Optional timeout parameter.
   */
  RCLCPP_PUBLIC
  void
  execute_ready_executables(
    executor::ExecutableList & exec_list,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Get list of TimerBase of all available timers.
  RCLCPP_PUBLIC
  void
  get_timer_list(executor::ExecutableList & exec_list);

  /// Get list of SubscriptionBase of all available subscriptions.
  RCLCPP_PUBLIC
  void
  get_subscription_list(executor::ExecutableList & exec_list);

  /// Get list of ServiceBase of all available services.
  RCLCPP_PUBLIC
  void
  get_service_list(executor::ExecutableList & exec_list);

  /// Get list of ClientBase of all available clients.
  RCLCPP_PUBLIC
  void
  get_client_list(executor::ExecutableList & exec_list);

  /// Get list of all available waitables.
  RCLCPP_PUBLIC
  void
  get_waitable_list(executor::ExecutableList & exec_list);

  /// Get available timers, subscribers, services, clients and waitables in ExecutableList struct.
  /**
   * \param[in] exec_list Structure that can hold subscriptionbases, timerbases, etc
   * \param[in] timeout Optional timeout parameter.
   */
  RCLCPP_PUBLIC
  void
  get_executable_list(
    executor::ExecutableList & executable_list,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Function to run collect_entities() and clean any invalid nodes.
  RCLCPP_PUBLIC
  void run_collect_entities();

  /// Function to add_handles_to_wait_set and wait for work and
  // block until the wait set is ready or until the timeout has been exceeded.
  RCLCPP_PUBLIC
  void refresh_wait_set(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Function to reallocate space for entities in the wait set.
  RCLCPP_PUBLIC
  void prepare_wait_set();


private:
  RCLCPP_DISABLE_COPY(StaticSingleThreadedExecutor)
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_
