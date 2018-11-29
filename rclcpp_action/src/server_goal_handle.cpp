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

namespace rclcpp_action
{
ServerGoalHandleBase::~ServerGoalHandleBase()
{
}

bool
ServerGoalHandleBase::is_cancel_request() const
{
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get goal handle state");
  }
  return GOAL_STATE_CANCELING == state;
}

void
ServerGoalHandleBase::_set_aborted()
{
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_ABORTED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_succeeded()
{
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_SUCCEEDED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerGoalHandleBase::_set_canceled()
{
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SET_CANCELED);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

std::shared_ptr<rcl_action_goal_handle_t>
ServerGoalHandleBase::get_rcl_handle() const
{
  return rcl_handle_;
}

void
ServerGoalHandleBase::_publish_feedback(std::shared_ptr<void> feedback_msg)
{
  rcl_ret_t ret = rcl_action_publish_feedback(rcl_server_.get(), feedback_msg.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to publish feedback");
  }
}
}  // namespace rclcpp_action
