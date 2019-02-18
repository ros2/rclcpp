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
#include <mutex>

#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action
{

/// Base class to interact with goals on a server.
/// \internal
/**
 *
 * This class in not be used directly by users writing an action server.
 * Instead users will be given an instance of `rclcpp_action::ServerGoalHandle<>`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ServerGoalHandleBase
{
public:
  /// Indicate if client has requested this goal be cancelled.
  /// \return true if a cancelation request has been accepted for this goal.
  RCLCPP_ACTION_PUBLIC
  bool
  is_canceling() const;

  /// Indicate if goal is pending or executing.
  /// \return false if goal has reached a terminal state.
  RCLCPP_ACTION_PUBLIC
  bool
  is_active() const;

  /// Indicate if goal is executing.
  /// \return true only if the goal is in an executing state.
  RCLCPP_ACTION_PUBLIC
  bool
  is_executing() const;

  RCLCPP_ACTION_PUBLIC
  virtual
  ~ServerGoalHandleBase();

protected:
  // -------------------------------------------------------------------------
  // API for communication between ServerGoalHandleBase and ServerGoalHandle<>

  /// \internal
  RCLCPP_ACTION_PUBLIC
  ServerGoalHandleBase(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle
  )
  : rcl_handle_(rcl_handle)
  {
  }

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  _set_aborted();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  _set_succeeded();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  _set_canceling();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  _set_canceled();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  _set_executing();

  /// Transition the goal to canceled state if it never reached a terminal state.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  try_canceling() noexcept;

  // End API for communication between ServerGoalHandleBase and ServerGoalHandle<>
  // -----------------------------------------------------------------------------

private:
  std::shared_ptr<rcl_action_goal_handle_t> rcl_handle_;
  mutable std::mutex rcl_handle_mutex_;
};

// Forward declare server
template<typename ActionT>
class Server;

/// Class to interact with goals on a server.
/**
 * Use this class to check the status of a goal as well as set the result.
 * This class is not meant to be created by a user, instead it is created when a goal has been
 * accepted.
 * The class `rclcpp_action::Server<>` will create an instance and give it to the user in their
 * `handle_accepted` callback.
 *
 * Internally, this class is responsible for coverting between the C++ action type and generic
 * types for `rclcpp_action::ServerGoalHandleBase`.
 */
template<typename ActionT>
class ServerGoalHandle : public ServerGoalHandleBase
{
public:
  /// Send an update about the progress of a goal.
  /**
   * This must only be called when the goal is executing.
   * If execution of a goal is deferred then `ServerGoalHandle<>::set_executing()` must be called
   * first.
   * `std::runtime_error` is raised if the goal is in any state besides executing.
   *
   * \param[in] feedback_msg the message to publish to clients.
   */
  void
  publish_feedback(std::shared_ptr<typename ActionT::Impl::FeedbackMessage> feedback_msg)
  {
    feedback_msg->goal_id.uuid = uuid_;
    publish_feedback_(feedback_msg);
  }

  // TODO(sloretz) which exception is raised?
  /// Indicate that a goal could not be reached and has been aborted.
  /**
   * Only call this if the goal was executing but cannot be completed.
   * This is a terminal state, no more methods should be called on a goal handle after this is
   * called.
   * An exception is raised if the goal is in any state besides executing.
   *
   * \param[in] result_msg the final result to send to clients.
   */
  void
  set_aborted(typename ActionT::Impl::GetResultService::Response::SharedPtr result_msg)
  {
    _set_aborted();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_ABORTED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// Indicate that a goal has been reached.
  /**
   * Only call this if the goal is executing and has reached the desired final state.
   * This is a terminal state, no more methods should be called on a goal handle after this is
   * called.
   * An exception is raised if the goal is in any state besides executing.
   *
   * \param[in] result_msg the final result to send to clients.
   */
  void
  set_succeeded(typename ActionT::Impl::GetResultService::Response::SharedPtr result_msg)
  {
    _set_succeeded();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// Indicate that a goal has been canceled.
  /**
   * Only call this if the goal is executing or pending, but has been canceled.
   * This is a terminal state, no more methods should be called on a goal handle after this is
   * called.
   * An exception is raised if the goal is in any state besides executing or pending.
   *
   * \param[in] result_msg the final result to send to clients.
   */
  void
  set_canceled(typename ActionT::Impl::GetResultService::Response::SharedPtr result_msg)
  {
    _set_canceled();
    result_msg->status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
    on_terminal_state_(uuid_, result_msg);
  }

  /// Indicate that the server is starting to execute a goal.
  /**
   * Only call this if the goal is pending.
   * An exception is raised if the goal is in any state besides pending.
   */
  void
  set_executing()
  {
    _set_executing();
    on_executing_(uuid_);
  }

  /// Get the user provided message describing the goal.
  const std::shared_ptr<const typename ActionT::Goal>
  get_goal() const
  {
    return goal_;
  }

  /// Get the unique identifier of the goal
  const GoalUUID &
  get_goal_id() const
  {
    return uuid_;
  }

  virtual ~ServerGoalHandle()
  {
    // Cancel goal if handle was allowed to destruct without reaching a terminal state
    if (try_canceling()) {
      auto null_result = std::make_shared<typename ActionT::Impl::GetResultService::Response>();
      null_result->status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
      on_terminal_state_(uuid_, null_result);
    }
  }

protected:
  /// \internal
  ServerGoalHandle(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle,
    GoalUUID uuid,
    std::shared_ptr<const typename ActionT::Goal> goal,
    std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state,
    std::function<void(const GoalUUID &)> on_executing,
    std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)> publish_feedback
  )
  : ServerGoalHandleBase(rcl_handle), goal_(goal), uuid_(uuid),
    on_terminal_state_(on_terminal_state), on_executing_(on_executing),
    publish_feedback_(publish_feedback)
  {
  }

  /// The user provided message describing the goal.
  const std::shared_ptr<const typename ActionT::Goal> goal_;

  /// A unique id for the goal request.
  const GoalUUID uuid_;

  friend Server<ActionT>;

  std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state_;
  std::function<void(const GoalUUID &)> on_executing_;
  std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)> publish_feedback_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
