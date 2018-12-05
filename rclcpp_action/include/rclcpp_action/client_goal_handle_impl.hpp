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

#include <memory>

#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action
{

template<typename ACTION>
ClientGoalHandle<ACTION>::ClientGoalHandle(
  const GoalInfo & info, FeedbackCallback callback)
: info_(info), result_future_(result_promise_.get_future()), feedback_callback_(callback)
{
}

template<typename ACTION>
ClientGoalHandle<ACTION>::~ClientGoalHandle()
{
}

template<typename ACTION>
const GoalID &
ClientGoalHandle<ACTION>::get_goal_id() const
{
  // return info_.goal_id;
  return info_.goal_id.uuid;
}

template<typename ACTION>
rclcpp::Time
ClientGoalHandle<ACTION>::get_goal_stamp() const
{
  return info_.stamp;
}


template<typename ACTION>
std::shared_future<typename ClientGoalHandle<ACTION>::Result>
ClientGoalHandle<ACTION>::async_result()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  if (!is_result_aware_) {
    throw exceptions::UnawareGoalHandleError();
  }
  return result_future_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_result(const Result & result)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = static_cast<int8_t>(result.code);
  result_promise_.set_value(result);
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_feedback_callback(FeedbackCallback callback)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  feedback_callback_ = callback;
}

template<typename ACTION>
int8_t
ClientGoalHandle<ACTION>::get_status()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return status_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_status(int8_t status)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = status;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::is_feedback_aware()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return feedback_callback_ != nullptr;
}

template<typename ACTION>
bool
ClientGoalHandle<ACTION>::is_result_aware()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return is_result_aware_;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::set_result_awareness(bool awareness)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  is_result_aware_ = awareness;
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::invalidate()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = GoalStatus::STATUS_UNKNOWN;
  result_promise_.set_exception(std::make_exception_ptr(
      exceptions::UnawareGoalHandleError()));
}

template<typename ACTION>
void
ClientGoalHandle<ACTION>::call_feedback_callback(
  ClientGoalHandle<ACTION>::SharedPtr shared_this,
  typename std::shared_ptr<const Feedback> feedback_message)
{
  if (shared_this.get() != this) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp_action"), "Sent feedback to wrong goal handle.");
    return;
  }
  std::lock_guard<std::mutex> guard(handle_mutex_);
  if (feedback_callback_ == nullptr) {
    // Normal, some feedback messages may arrive after the goal result.
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"), "Received feedback but goal ignores it.");
    return;
  }
  feedback_callback_(shared_this, feedback_message);
}

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
