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

#ifndef RCLCPP__GUARD_CONDITION_HPP_
#define RCLCPP__GUARD_CONDITION_HPP_

#include <atomic>

#include "rcl/guard_condition.h"

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// A condition that can be waited on in a single wait set and asynchronously triggered.
class GuardCondition
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(GuardCondition)

  // TODO(wjwwood): support custom allocator, maybe restrict to polymorphic allocator
  /// Construct the guard condition, optionally specifying which Context to use.
  /**
   * \param[in] context Optional custom context to be used.
   *   Defaults to using the global default context singleton.
   *   Shared ownership of the context is held with the guard condition until
   *   destruction.
   * \param[in] guard_condition_options Optional guard condition options to be used.
   *   Defaults to using the default guard condition options.
   * \throws std::invalid_argument if the context is nullptr.
   * \throws rclcpp::exceptions::RCLError based exceptions when underlying
   *   rcl functions fail.
   */
  RCLCPP_PUBLIC
  explicit GuardCondition(
    rclcpp::Context::SharedPtr context =
    rclcpp::contexts::get_global_default_context(),
    rcl_guard_condition_options_t guard_condition_options =
    rcl_guard_condition_get_default_options());

  RCLCPP_PUBLIC
  virtual
  ~GuardCondition();

  /// Return the context used when creating this guard condition.
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr
  get_context() const;

  /// Return the underlying rcl guard condition structure.
  RCLCPP_PUBLIC
  rcl_guard_condition_t &
  get_rcl_guard_condition();

  /// Return the underlying rcl guard condition structure.
  RCLCPP_PUBLIC
  const rcl_guard_condition_t &
  get_rcl_guard_condition() const;

  /// Notify the wait set waiting on this condition, if any, that the condition had been met.
  /**
   * This function is thread-safe, and may be called concurrently with waiting
   * on this guard condition in a wait set.
   *
   * \throws rclcpp::exceptions::RCLError based exceptions when underlying
   *   rcl functions fail.
   */
  RCLCPP_PUBLIC
  void
  trigger();

  /// Exchange the "in use by wait set" state for this guard condition.
  /**
   * This is used to ensure this guard condition is not used by multiple
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

  /// Adds the guard condition to a waitset
  /**
   * This function is thread-safe.
   * \param[in] wait_set pointer to a wait set where to add the guard condition
   */
  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set);

  RCLCPP_PUBLIC
  void
  set_on_trigger_callback(std::function<void(size_t)> callback);

protected:
  rclcpp::Context::SharedPtr context_;
  rcl_guard_condition_t rcl_guard_condition_;
  std::atomic<bool> in_use_by_wait_set_{false};
  std::recursive_mutex reentrant_mutex_;
  std::function<void(size_t)> on_trigger_callback_{nullptr};
  size_t unread_count_{0};
  rcl_wait_set_t * wait_set_{nullptr};
};

}  // namespace rclcpp

#endif  // RCLCPP__GUARD_CONDITION_HPP_
