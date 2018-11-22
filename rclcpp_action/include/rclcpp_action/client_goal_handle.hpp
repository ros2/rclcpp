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
#include <mutex>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_info.hpp"
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

  std::future<typename ACTION::Result>
  async_result();

  std::function<void()>
  get_feedback_callback();

  action_msgs::msg::GoalStatus
  get_status();

  bool
  is_feedback_aware();

  bool
  is_result_aware();

  void
  set_result_awareness(bool awareness);

  bool
  is_valid();

  void
  invalidate();

private:
  // The templated Server creates goal handles
  friend Client<ACTION>;

  ClientGoalHandle(const action_msgs::msg::GoalInfo info);

  void
  set_feedback_callback(std::function<void()> callback);

  void
  set_status(action_msgs::msg::GoalStatus status);

  void
  set_result(typename ACTION::Result result);

  action_msgs::msg::GoalInfo info_;
  action_msgs::msg::GoalStatus status_;

  std::function<void()> feedback_callback_;

  bool is_result_aware_;
  std::promise<typename ACTION::Result> result_;

  bool is_handler_valid_;

  std::mutex handler_mutex_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/client_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
