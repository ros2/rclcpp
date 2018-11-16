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

#ifndef RCLCPP_ACTION__SERVER_GOAL_HANDLE_IMPL_HPP_
#define RCLCPP_ACTION__SERVER_GOAL_HANDLE_IMPL_HPP_

#include <rcl_action/goal_handle.h>
#include <rcl_action/types.h>

namespace rclcpp_action
{
template<typename ACTION>
ServerGoalHandle<ACTION>::ServerGoalHandle(
  rcl_action_server_t * rcl_server,
  const typename ACTION::Goal goal,
  rcl_action_goal_handle_t * rcl_handle
)
: rcl_server_(rcl_server), goal_(goal), rcl_handle_(rcl_handle)
{
}

template<typename ACTION>
ServerGoalHandle<ACTION>::~ServerGoalHandle()
{
}

template<typename ACTION>
bool
ServerGoalHandle<ACTION>::is_cancel_request() const
{
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  if (RCL_RET_OK != rcl_action_goal_handle_get_status(rcl_handle_, &state)) {
    // TODO(sloretz) more specific exception
    throw std::runtime_error("Failed to get goal handle state");
  }
  return GOAL_STATE_CANCELING == state;
}

template<typename ACTION>
void
ServerGoalHandle<ACTION>::publish_feedback(const typename ACTION::Feedback * feedback_msg)
{
  // TODO(sloretz) what is ros_message and how does IntraProcessmessage come into play?
  // if (RCL_RET_OK != rcl_action_publish_feedback(rcl_server_, ros_message) {
  //   // TODO(sloretz) more specific exception
  //   throw std::runtime_error("Failed to publish feedback");
  // }
  throw std::runtime_error("Failed to publish feedback");
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_IMPL_HPP_
