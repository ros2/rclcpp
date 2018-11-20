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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_

#include <rcl_action/action_client.h>

#include <functional>
#include <memory>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
// Forward declarations
template<typename ACTION>
class Client;

template<typename ACTION>
class ClientGoalHandle
{
public:
  virtual ~ClientGoalHandle();

  std::future<bool>
  async_cancel();

  std::future<typename ACTION::Result>
  async_result();

private:
  // The templated Server creates goal handles
  friend Client<ACTION>;

  ClientGoalHandle(rcl_action_client_t * rcl_client, const rcl_action_goal_info_t rcl_info);

  // TODO(sloretz) shared pointer to keep rcl_client_ alive while goal handles are alive
  rcl_action_client_t * rcl_client_;
  rcl_action_goal_info_t rcl_info_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/client_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
