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
ClientGoalHandle<ACTION>::ClientGoalHandle(const action_msgs::msg::GoalInfo info)
: info_(info),
  feedback_callback_(nullptr),
  is_result_aware_(false),
  is_handler_valid_(true)
{
}

template<typename ACTION>
ClientGoalHandle<ACTION>::~ClientGoalHandle()
{
}

template<typename ACTION>
std::future<typename ACTION::Result>
ClientGoalHandle<ACTION>::async_result()
{
  return result_.get_future();
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_result(typename ACTION::Result result)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  result_.set_value(result);
}

template<typename ACTION>
std::function<void()>
ClientGoalHandle<ACTION>::get_feedback_callback()
{
  return feedback_callback_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_feedback_callback(std::function<void()> callback)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  feedback_callback_ = callback;
}

template<typename ACTION>
action_msgs::msg::GoalStatus
ClientGoalHandle<ACTION>::get_status()
{
  return status_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_status(action_msgs::msg::GoalStatus status)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  status_ = status;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::is_feedback_aware()
{
  return feedback_callback_;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::is_result_aware()
{
  return is_result_aware_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_result_awareness(bool awareness)
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  is_result_aware_ = awareness;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::is_valid()
{
  return is_handler_valid_;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::invalidate()
{
  std::lock_guard<std::mutex> guard(handler_mutex_);
  is_handler_valid_ = false;
}

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
