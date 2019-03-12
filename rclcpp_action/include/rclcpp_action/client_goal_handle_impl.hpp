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

#include "rclcpp/logging.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action
{

template<typename ActionT>
ClientGoalHandle<ActionT>::ClientGoalHandle(
  const GoalInfo & info, FeedbackCallback callback)
: info_(info), result_future_(result_promise_.get_future()), feedback_callback_(callback)
{
}

template<typename ActionT>
ClientGoalHandle<ActionT>::~ClientGoalHandle()
{
}

template<typename ActionT>
const GoalUUID &
ClientGoalHandle<ActionT>::get_goal_id() const
{
  // return info_.goal_id;
  return info_.goal_id.uuid;
}

template<typename ActionT>
rclcpp::Time
ClientGoalHandle<ActionT>::get_goal_stamp() const
{
  return info_.stamp;
}

template<typename ActionT>
std::shared_future<typename ClientGoalHandle<ActionT>::WrappedResult>
ClientGoalHandle<ActionT>::async_result()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  if (!is_result_aware_) {
    throw exceptions::UnawareGoalHandleError();
  }
  return result_future_;
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::set_result(const WrappedResult & wrapped_result)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = static_cast<int8_t>(wrapped_result.code);
  result_promise_.set_value(wrapped_result);
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::set_feedback_callback(FeedbackCallback callback)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  feedback_callback_ = callback;
}

template<typename ActionT>
int8_t
ClientGoalHandle<ActionT>::get_status()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return status_;
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::set_status(int8_t status)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = status;
}

template<typename ActionT>
bool
ClientGoalHandle<ActionT>::is_feedback_aware()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return feedback_callback_ != nullptr;
}

template<typename ActionT>
bool
ClientGoalHandle<ActionT>::is_result_aware()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  return is_result_aware_;
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::set_result_awareness(bool awareness)
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  is_result_aware_ = awareness;
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::invalidate()
{
  std::lock_guard<std::mutex> guard(handle_mutex_);
  status_ = GoalStatus::STATUS_UNKNOWN;
  result_promise_.set_exception(std::make_exception_ptr(
      exceptions::UnawareGoalHandleError()));
}

template<typename ActionT>
void
ClientGoalHandle<ActionT>::call_feedback_callback(
  typename ClientGoalHandle<ActionT>::SharedPtr shared_this,
  typename std::shared_ptr<const Feedback> feedback_message)
{
  if (shared_this.get() != this) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp_action"), "Sent feedback to wrong goal handle.");
    return;
  }
  std::lock_guard<std::mutex> guard(handle_mutex_);
  if (nullptr == feedback_callback_) {
    // Normal, some feedback messages may arrive after the goal result.
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"), "Received feedback but goal ignores it.");
    return;
  }
  feedback_callback_(shared_this, feedback_message);
}

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
