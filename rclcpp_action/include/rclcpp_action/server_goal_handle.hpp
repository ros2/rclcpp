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
// Forward declarations
template<typename ACTION>
class Server;

template<typename ACTION>
class ServerGoalHandle
{
public:
  virtual ~ServerGoalHandle();

  /// Indicate if client has requested this goal be cancelled.
  /// \return true if a cancelation request has been accepted for this goal.
  bool
  is_cancel_request() const;

  /// Send an update about the progress of a goal.
  void publish_feedback(const typename ACTION::Feedback * feedback_msg);

  // TODO(sloretz) examples has this attribute as 'goal'
  /// The original request message describing the goal.
  const typename ACTION::Goal goal_;

  // TODO(sloretz) `set_cancelled`, `set_succeeded`, `set_aborted`

private:
  // The templated Server creates goal handles
  friend Server<ACTION>;

  RCLCPP_ACTION_PUBLIC
  ServerGoalHandle(
    rcl_action_server_t * rcl_server, const typename ACTION::Goal goal,
    rcl_action_goal_handle_t * rcl_handle);

  // TODO(sloretz) shared pointer to keep rcl_server_ alive while goal handles are alive
  rcl_action_server_t * rcl_server_;
  rcl_action_goal_handle_t * rcl_handle_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/server_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
