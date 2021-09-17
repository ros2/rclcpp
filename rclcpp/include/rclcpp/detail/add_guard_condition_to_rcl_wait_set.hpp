// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__ADD_GUARD_CONDITION_TO_RCL_WAIT_SET_HPP_
#define RCLCPP__DETAIL__ADD_GUARD_CONDITION_TO_RCL_WAIT_SET_HPP_

#include "rclcpp/guard_condition.hpp"

namespace rclcpp
{
namespace detail
{

/// Adds the guard condition to a waitset
/**
 * \param[in] wait_set reference to a wait set where to add the guard condition
 * \param[in] guard_condition reference to the guard_condition to be added
 */
RCLCPP_PUBLIC
void
add_guard_condition_to_rcl_wait_set(
  rcl_wait_set_t & wait_set,
  const rclcpp::GuardCondition & guard_condition);

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__ADD_GUARD_CONDITION_TO_RCL_WAIT_SET_HPP_
