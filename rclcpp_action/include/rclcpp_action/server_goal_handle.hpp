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

class ServerGoalHandleBase
{
public:
  /// Indicate if client has requested this goal be cancelled.
  /// \return true if a cancelation request has been accepted for this goal.
  bool
  is_cancel_request() const;

  virtual
  ~ServerGoalHandleBase();

protected:
  ServerGoalHandleBase(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle
  )
  : rcl_server_(rcl_server), rcl_handle_(rcl_handle)
  {
  }

  void
  publish_feedback(std::shared_ptr<void> feedback_msg);

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
    publish_feedback(std::static_pointer_cast<void>(feedback_msg));
  }

  // TODO(sloretz) `set_cancelled`, `set_succeeded`, `set_aborted`

  /// The original request message describing the goal.
  const std::shared_ptr<const typename ACTION::Goal> goal_;

  /// A unique id for the goal request.
  const std::array<uint8_t, 16> uuid_;

protected:
  ServerGoalHandle(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle,
    std::array<uint8_t, 16> uuid,
    std::shared_ptr<const typename ACTION::Goal> goal
  )
  : ServerGoalHandleBase(rcl_server, rcl_handle), goal_(goal), uuid_(uuid)
  {
  }

  friend Server<ACTION>;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
