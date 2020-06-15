// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAITABLE_HPP_
#define RCLCPP__WAITABLE_HPP_

#include <atomic>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/wait.h"

namespace rclcpp
{

class Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Waitable)

  RCLCPP_PUBLIC
  virtual ~Waitable() = default;

  /// Get the number of ready subscriptions
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more subscriptions.
   * \return The number of subscriptions associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_subscriptions();

  /// Get the number of ready timers
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more timers.
   * \return The number of timers associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_timers();

  /// Get the number of ready clients
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more clients.
   * \return The number of clients associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_clients();

  /// Get the number of ready events
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more events.
   * \return The number of events associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_events();

  /// Get the number of ready services
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more services.
   * \return The number of services associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_services();

  /// Get the number of ready guard_conditions
  /**
   * Returns a value of 0 by default.
   * This should be overridden if the Waitable contains one or more guard_conditions.
   * \return The number of guard_conditions associated with the Waitable.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  get_number_of_ready_guard_conditions();

  /// Add the Waitable to a wait set.
  /**
   * \param[in] wait_set A handle to the wait set to add the Waitable to.
   * \return `true` if the Waitable is added successfully, `false` otherwise.
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_add_*()
   */
  RCLCPP_PUBLIC
  virtual
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) = 0;

  /// Check if the Waitable is ready.
  /**
   * The input wait set should be the same that was used in a previously call to
   * `add_to_wait_set()`.
   * The wait set should also have been previously waited on with `rcl_wait()`.
   *
   * \param[in] wait_set A handle to the wait set the Waitable was previously added to
   *   and that has been waited on.
   * \return `true` if the Waitable is ready, `false` otherwise.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  is_ready(rcl_wait_set_t * wait_set) = 0;

  /// Execute any entities of the Waitable that are ready.
  /**
   * Before calling this method, the Waitable should be added to a wait set,
   * waited on, and then updated.
   *
   * Example usage:
   *
   * ```cpp
   * // ... create a wait set and a Waitable
   * // Add the Waitable to the wait set
   * bool add_ret = waitable.add_to_wait_set(wait_set);
   * // ... error handling
   * // Wait
   * rcl_ret_t wait_ret = rcl_wait(wait_set);
   * // ... error handling
   * // Update the Waitable
   * waitable.update(wait_set);
   * // Execute any entities of the Waitable that may be ready
   * waitable.execute();
   * ```
   */
  RCLCPP_PUBLIC
  virtual
  void
  execute() = 0;

  /// Exchange the "in use by wait set" state for this timer.
  /**
   * This is used to ensure this timer is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool
  exchange_in_use_by_wait_set_state(bool in_use_state);

private:
  std::atomic<bool> in_use_by_wait_set_{false};
};  // class Waitable

}  // namespace rclcpp

#endif  // RCLCPP__WAITABLE_HPP_
