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

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/wait.h"

namespace rclcpp
{

class Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Waitable)

  Waitable() {}
  virtual ~Waitable() {}

  /// Get the number of ready subscriptions
  virtual size_t get_number_of_ready_subscriptions();

  /// Get the number of ready timers
  virtual size_t get_number_of_ready_timers();

  /// Get the number of ready clients
  virtual size_t get_number_of_ready_clients();

  /// Get the number of ready services
  virtual size_t get_number_of_ready_services();

  /// Get the number of ready guard_conditions
  virtual size_t get_number_of_ready_guard_conditions();

  // TODO(jacobperron): smart pointer?
  /// Add the Waitable to a wait set.
  /**
   * \param[in] wait_set A handle to the wait set to add the Waitable to.
   * \return `true` if the Waitable is added successfully, `false` otherwise.
   */
  virtual bool add_to_wait_set(rcl_wait_set_t * wait_set) = 0;

  // TODO(jacobperron): Rename to `is_ready()`?
  /// Check the Waitable can execute something.
  /**
   * \param[in] wait_set A handle to the wait set the Waitable was previously added to.
   * \return `true` if the Waitable can execute, `false` otherwise.
   */
  virtual bool can_execute(rcl_wait_set_t *) = 0;

  // TODO(jacobperron): pass optional handle to wait set?
  virtual void execute() = 0;
};  // class Waitable

}  // namespace rclcpp

#endif  // RCLCPP__WAITABLE_HPP_
