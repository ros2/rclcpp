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

#ifndef RCLCPP__EXECUTOR_POLICIES__TIMER_FAVORING_PRIORITY_QUEUE_HPP_
#define RCLCPP__EXECUTOR_POLICIES__TIMER_FAVORING_PRIORITY_QUEUE_HPP_

#include "rclcpp/any_executable.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executor_policies/scheduling_result.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"

namespace rclcpp
{
namespace executor_policies
{

/// A naive scheduling policy which selects Timers first, then Subscriptions and other items.
/**
 * Items are executed in the order they were added, favoring Timers, then
 * Subscriptions, Service Servers, Service Clients, and finally Waitables.
 * All Timers are executed before any Subscriptions, and all Subscriptions
 * before any Service Servers, and so on.
 *
 * User guard conditions are not yet supported by the Executor and so all guard
 * condition are used by the executor itself and are handled before this policy
 * is consulted.
 * Therefore, guard conditions are ignored for the purposes of scheduling.
 *
 * This is a naive policy, but is the default until a better one is implemented.
 */
class TimerFavoringPriorityQueue
{
public:
  explicit TimerFavoringPriorityQueue(const rclcpp::ExecutorOptions &) {}

  /// Select which item should next be executed, and indicate if waiting should resume.
  /**
   * Selects which item to be executed next, assign it to the any_executable, or
   * assigning nullptr if no work should be done right now.
   *
   * This method is called by the executor after waiting on a wait set, in
   * order to determine what to execute next based on the result.
   *
   * Additionally, if returning SchedulingResult::ContinueExecuting then the
   * executor will call this function again without waiting on the wait set, or
   * if returning SchedulingResult::WaitForWork then the executor will wait on
   * the wait set again after executing the selected any_executable, or
   * immediately if any_executable was assigned nullptr.
   */
  template<class WaitSetT>
  rclcpp::executor_policies::SchedulingResult
  schedule_next_any_executable(
    const WaitResult<WaitSetT> & wait_result,
    rclcpp::AnyExecutable & any_executable)
  {
    // Explicitly ignore guard conditions.
    // Check Timers for being ready.

  }
};

}  // namespace executor_policies
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_POLICIES__TIMER_FAVORING_PRIORITY_QUEUE_HPP_
