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

#include <rcl_action/types.h>
#include <rcl_action/goal_handle.h>

#include <action_msgs/msg/goal_status.hpp>

#include <functional>
#include <memory>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{

class ServerGoalHandleBase
{
public:
  /// Indicate if client has requested this goal be cancelled.
  /// \return true if a cancelation request has been accepted for this goal.
  bool
  is_cancel_request() const;

  /// Indicate if goal is being worked on.
  /// \return false if goal has reached a terminal state.
  bool
  is_active() const;

  virtual
  ~ServerGoalHandleBase();

  std::shared_ptr<rcl_action_goal_handle_t>
  get_rcl_handle() const;

protected:
  ServerGoalHandleBase(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle
  )
  : rcl_server_(rcl_server), rcl_handle_(rcl_handle)
  {
  }

  void
  _publish_feedback(std::shared_ptr<void> feedback_msg);

  void
  _set_aborted();

  void
  _set_succeeded();

  void
  _set_canceled();

private:
  std::shared_ptr<rcl_action_server_t> rcl_server_;
  std::shared_ptr<rcl_action_goal_handle_t> rcl_handle_;
};

// Forward declar server
template<typename ACTION>
class Server;

template<typename ACTION>
class ServerGoalHandle : public ServerGoalHandleBase
{
public:
  virtual ~ServerGoalHandle() = default;

  /// Send an update about the progress of a goal.
  void
  publish_feedback(std::shared_ptr<typename ACTION::Feedback> feedback_msg)
  {
    _publish_feedback(std::static_pointer_cast<void>(feedback_msg));
  }

  /// Indicate that a goal could not be reached and has been aborted.
  void
  set_aborted(typename ACTION::Result::SharedPtr result_msg)
  {
    _set_aborted();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_ABORTED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// Indicate that a goal has been reached.
  void
  set_succeeded(typename ACTION::Result::SharedPtr result_msg)
  {
    _set_succeeded();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// Indicate that a goal has been canceled.
  void
  set_canceled(typename ACTION::Result::SharedPtr result_msg)
  {
    _set_canceled();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// The original request message describing the goal.
  const std::shared_ptr<const typename ACTION::Goal> goal_;

  /// A unique id for the goal request.
  const std::array<uint8_t, 16> uuid_;

protected:
  ServerGoalHandle(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle,
    std::array<uint8_t, 16> uuid,
    std::shared_ptr<const typename ACTION::Goal> goal,
    std::function<void(const std::array<uint8_t, 16>&, std::shared_ptr<void>)> on_terminal_state
  )
  : ServerGoalHandleBase(rcl_server, rcl_handle), goal_(goal), uuid_(uuid),
    on_terminal_state_(on_terminal_state)
  {
  }

  friend Server<ACTION>;

  std::function<void(const std::array<uint8_t, 16>&, std::shared_ptr<void>)> on_terminal_state_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
