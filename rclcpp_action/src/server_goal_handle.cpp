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
ServerGoalHandleBase::publish_feedback(std::shared_ptr<void> feedback_msg)
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
