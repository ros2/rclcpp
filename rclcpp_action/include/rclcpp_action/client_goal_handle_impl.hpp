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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_

#include <rcl_action/types.h>

namespace rclcpp_action
{
template<typename ACTION>
ClientGoalHandle<ACTION>::ClientGoalHandle(
  rcl_action_client_t * rcl_client,
  const rcl_action_goal_info_t rcl_info
)
: rcl_client_(rcl_client), rcl_info_(rcl_info)
{
}

template<typename ACTION>
ClientGoalHandle<ACTION>::~ClientGoalHandle()
{
}

template<typename ACTION>
std::future<bool>
ClientGoalHandle<ACTION>::async_cancel()
{
  throw std::runtime_error("Failed to cancel goal");
}

template<typename ACTION>
std::future<typename ACTION::Result>
ClientGoalHandle<ACTION>::async_result()
{
  throw std::runtime_error("Failed to get result future");
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
