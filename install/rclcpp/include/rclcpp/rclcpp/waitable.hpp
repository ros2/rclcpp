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
#include <functional>
#include <memory>

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
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_add_*()
   */
  RCLCPP_PUBLIC
  virtual
  void
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

  /// Take the data so that it can be consumed with `execute`.
  /**
   * NOTE: take_data is a partial fix to a larger design issue with the
   * multithreaded executor. This method is likely to be removed when
   * a more permanent fix is implemented. A longterm fix is currently
   * being discussed here: https://github.com/ros2/rclcpp/pull/1276
   *
   * This method takes the data from the underlying data structure and
   * writes it to the void shared pointer `data` that is passed into the
   * method. The `data` can then be executed with the `execute` method.
   *
   * Before calling this method, the Waitable should be added to a wait set,
   * waited on, and then updated.
   *
   * Example usage:
   *
   * ```cpp
   * // ... create a wait set and a Waitable
   * // Add the Waitable to the wait set
   * waitable.add_to_wait_set(wait_set);
   * // Wait
   * rcl_ret_t wait_ret = rcl_wait(wait_set);
   * // ... error handling
   * // Update the Waitable
   * waitable.update(wait_set);
   * // Execute any entities of the Waitable that may be ready
   * std::shared_ptr<void> data = waitable.take_data();
   * ```
   */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<void>
  take_data() = 0;

  /// Take the data so that it can be consumed with `execute`.
  /**
   * This function allows to specify an entity ID to take the data from.
   * Entity IDs are identifiers that can be defined by waitable-derived
   * classes that are composed of several distinct entities.
   * The main use-case is in conjunction with the listener APIs.
   *
   * \param[in] id the id of the entity from which to take
   * \returns the type-erased data taken from entity specified
   *
   * \sa rclcpp::Waitable::take_data
   * \sa rclcpp::Waitable::set_on_ready_callback
   */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id);

  /// Execute data that is passed in.
  /**
   * Before calling this method, the Waitable should be added to a wait set,
   * waited on, and then updated - and the `take_data` method should be
   * called.
   *
   * Example usage:
   *
   * ```cpp
   * // ... create a wait set and a Waitable
   * // Add the Waitable to the wait set
   * waitable.add_to_wait_set(wait_set);
   * // Wait
   * rcl_ret_t wait_ret = rcl_wait(wait_set);
   * // ... error handling
   * // Update the Waitable
   * waitable.update(wait_set);
   * // Execute any entities of the Waitable that may be ready
   * std::shared_ptr<void> data = waitable.take_data();
   * waitable.execute(data);
   * ```
   */
  RCLCPP_PUBLIC
  virtual
  void
  execute(std::shared_ptr<void> & data) = 0;

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

  /// Set a callback to be called whenever the waitable becomes ready.
  /**
   * The callback receives a size_t which is the number of times the waitable was ready
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if waitable was triggered before any
   * callback was set.
   *
   * The callback also receives an int identifier argument.
   * This is needed because a Waitable may be composed of several distinct entities,
   * such as subscriptions, services, etc.
   * The application should provide a generic callback function that will be then
   * forwarded by the waitable to all of its entities.
   * Before forwarding, a different value for the identifier argument will be
   * bond to the function.
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * Note: this function must be overridden with a proper implementation
   * by the custom classes who inherit from rclcpp::Waitable if they want to use it.
   *
   * \sa rclcpp::Waitable::clear_on_ready_callback
   *
   * \param[in] callback functor to be called when the waitable becomes ready
   */
  RCLCPP_PUBLIC
  virtual
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback);

  /// Unset any callback registered via set_on_ready_callback.
  /**
   * Note: this function must be overridden with a proper implementation
   * by the custom classes who inherit from rclcpp::Waitable if they want to use it.
   */
  RCLCPP_PUBLIC
  virtual
  void
  clear_on_ready_callback();

private:
  std::atomic<bool> in_use_by_wait_set_{false};
};  // class Waitable

}  // namespace rclcpp

#endif  // RCLCPP__WAITABLE_HPP_
