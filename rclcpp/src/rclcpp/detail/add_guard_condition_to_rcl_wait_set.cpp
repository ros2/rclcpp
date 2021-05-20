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

#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/exceptions.hpp"

namespace rclcpp
{
namespace detail
{

void
add_guard_condition_to_rcl_wait_set(
  rcl_wait_set_t & wait_set,
  const rclcpp::GuardCondition & guard_condition)
{
  const auto & gc = guard_condition.get_rcl_guard_condition();

  rcl_ret_t ret = rcl_wait_set_add_guard_condition(&wait_set, &gc, NULL);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "failed to add guard condition to wait set");
  }
}

}  // namespace detail
}  // namespace rclcpp
