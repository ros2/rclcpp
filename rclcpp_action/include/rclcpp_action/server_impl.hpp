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

#ifndef RCLCPP_ACTION__SERVER_IMPL_HPP_
#define RCLCPP_ACTION__SERVER_IMPL_HPP_

#include <rcl_action/types.h>

#include <memory>

namespace rclcpp_action
{
template<typename ACTION>
Server<ACTION>::GoalHandle::GoalHandle(
  std::shared_ptr<rcl_action_server_t> rcl_server,
  const typename ACTION::Goal goal,
  std::shared_ptr<rcl_action_goal_handle_t> rcl_handle
)
: ServerGoalHandle<ACTION, Server<ACTION>::GoalHandle>(goal),
  rcl_server_(rcl_server), rcl_handle_(rcl_handle)
{
}

template<typename ACTION>
Server<ACTION>::GoalHandle::~GoalHandle()
{
}

template<typename ACTION>
bool
Server<ACTION>::GoalHandle::is_cancel_request() const
{
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  if (RCL_RET_OK != rcl_action_goal_handle_get_status(rcl_handle_.get(), &state)) {
    // TODO(sloretz) more specific exception
    throw std::runtime_error("Failed to get goal handle state");
  }
  return GOAL_STATE_CANCELING == state;
}

template<typename ACTION>
void
Server<ACTION>::GoalHandle::publish_feedback(const typename ACTION::Feedback * feedback_msg)
{
  (void)feedback_msg;
  // TODO(sloretz) what is ros_message and how does IntraProcessmessage come into play?
  // if (RCL_RET_OK != rcl_action_publish_feedback(rcl_server_, ros_message) {
  //   // TODO(sloretz) more specific exception
  //   throw std::runtime_error("Failed to publish feedback");
  // }
  throw std::runtime_error("Failed to publish feedback");
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_IMPL_HPP_
