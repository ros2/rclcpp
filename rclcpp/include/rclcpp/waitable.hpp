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

struct NumberOfWaitables
{
  size_t number_of_subscriptions;
  size_t number_of_services;
  size_t number_of_clients;
  size_t number_of_timers;
  size_t number_of_guard_conditions;
  NumberOfWaitables()
  : number_of_subscriptions(0),
    number_of_services(0),
    number_of_clients(0),
    number_of_timers(0),
    number_of_guard_conditions(0)
  {}
};

class Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Waitable)

  Waitable() {}
  virtual ~Waitable() {}

  /// Get the number of waitable entities.
  virtual NumberOfWaitables get_number_of_entities() = 0;

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
