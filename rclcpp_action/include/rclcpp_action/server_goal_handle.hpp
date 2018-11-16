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

#ifndef RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
#define RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_

#include <rcl_action/goal_handle.h>

#include <functional>
#include <memory>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
template<typename ACTION, typename DERIVED>
class ServerGoalHandle
{
public:
  explicit ServerGoalHandle(typename ACTION::Goal goal)
  : goal_(goal)
  {
  }

  virtual ~ServerGoalHandle()
  {
  }

  bool
  is_cancel_request() const
  {
    return static_cast<DERIVED *>(this)->is_cancel_request();
  }

  void
  publish_feedback(const typename ACTION::Feedback * feedback_msg)
  {
    return static_cast<DERIVED *>(this)->publish_feedback(feedback_msg);
  }

  // TODO(sloretz) `set_cancelled`, `set_succeeded`, `set_aborted`

  // TODO(sloretz) examples has this attribute as 'goal'
  /// The original request message describing the goal.
  const typename ACTION::Goal goal_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
