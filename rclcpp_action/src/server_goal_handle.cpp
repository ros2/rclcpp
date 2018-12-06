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
ServerGoalHandleBase::_set_aborted()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_ABORTED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_succeeded()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_SUCCEEDED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_canceling()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_canceled()
{
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_CANCELED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_executing()
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
  // Check if the goal reached a terminal state already
  const bool active = rcl_action_goal_handle_is_active(rcl_handle_.get());
  if (!active) {
    return false;
  }

  rcl_ret_t ret;

  // Get the current state
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    return false;
  }

  // If it's not already canceling then transition to that state
  if (GOAL_STATE_CANCELING != state) {
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL);
    if (RCL_RET_OK != ret) {
      return false;
    }
  }

  // Get the state again
  ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    return false;
  }

  // If it's canceling, cancel it
  if (GOAL_STATE_CANCELING == state) {
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_CANCELED);
    return RCL_RET_OK == ret;
  }

  return false;
}
}  // namespace rclcpp_action
