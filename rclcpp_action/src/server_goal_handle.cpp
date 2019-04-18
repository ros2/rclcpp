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

#include <rcl_action/action_server.h>
#include <rcl_action/goal_handle.h>

#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/exceptions.hpp>

#include <memory>

namespace rclcpp_action
{
ServerGoalHandleBase::~ServerGoalHandleBase()
{
}

bool
ServerGoalHandleBase::is_canceling() const
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get goal handle state");
  }
  return GOAL_STATE_CANCELING == state;
}

bool
ServerGoalHandleBase::is_active() const
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  return rcl_action_goal_handle_is_active(rcl_handle_.get());
}

bool
ServerGoalHandleBase::is_executing() const
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get goal handle state");
  }
  return GOAL_STATE_EXECUTING == state;
}

void
ServerGoalHandleBase::_abort()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_ABORT);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_succeed()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SUCCEED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_cancel_goal()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL_GOAL);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_canceled()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCELED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_execute()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_EXECUTE);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

bool
ServerGoalHandleBase::try_canceling() noexcept
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret;
  // Check if the goal is cancelable
  const bool is_cancelable = rcl_action_goal_handle_is_cancelable(rcl_handle_.get());
  if (is_cancelable) {
    // Transition to CANCELING
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL_GOAL);
    if (RCL_RET_OK != ret) {
      return false;
    }
  }

  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  // Get the current state
  ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    return false;
  }

  // If it's canceling, cancel it
  if (GOAL_STATE_CANCELING == state) {
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCELED);
    return RCL_RET_OK == ret;
  }

  return false;
}
}  // namespace rclcpp_action
