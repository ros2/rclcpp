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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_

#include <functional>
#include <future>
#include <memory>

#include "rcl_action/action_client.h"

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

#include "rcpputils/mutex.hpp"

namespace rclcpp_action
{
/// The possible statuses that an action goal can finish with.
enum class ResultCode : int8_t
{
  UNKNOWN = action_msgs::msg::GoalStatus::STATUS_UNKNOWN,
  SUCCEEDED = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
  CANCELED = action_msgs::msg::GoalStatus::STATUS_CANCELED,
  ABORTED = action_msgs::msg::GoalStatus::STATUS_ABORTED
};


// Forward declarations
template<typename ActionT>
class Client;

/// Class for interacting with goals sent from action clients.
/**
 * Use this class to check the status of a goal as well as get the result.
 *
 * This class is not meant to be created by a user, instead it is created when a goal has been
 * accepted.
 * A `Client` will create an instance and return it to the user (via a future) after calling
 * `Client::async_send_goal`.
 */
template<typename ActionT>
class ClientGoalHandle
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientGoalHandle)

  // A wrapper that defines the result of an action
  struct WrappedResult
  {
    /// The unique identifier of the goal
    GoalUUID goal_id;
    /// A status to indicate if the goal was canceled, aborted, or suceeded
    ResultCode code;
    /// User defined fields sent back with an action
    typename ActionT::Result::SharedPtr result;
  };

  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using FeedbackCallback =
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
  using ResultCallback = std::function<void (const WrappedResult & result)>;

  virtual ~ClientGoalHandle();

  /// Get the unique ID for the goal.
  const GoalUUID &
  get_goal_id() const;

  /// Get the time when the goal was accepted.
  rclcpp::Time
  get_goal_stamp() const;

  /// Get the goal status code.
  int8_t
  get_status();

  /// Check if an action client has subscribed to feedback for the goal.
  bool
  is_feedback_aware();

  /// Check if an action client has requested the result for the goal.
  bool
  is_result_aware();

private:
  // The templated Client creates goal handles
  friend class Client<ActionT>;

  ClientGoalHandle(
    const GoalInfo & info,
    FeedbackCallback feedback_callback,
    ResultCallback result_callback);

  void
  set_feedback_callback(FeedbackCallback callback);

  void
  set_result_callback(ResultCallback callback);

  void
  call_feedback_callback(
    typename ClientGoalHandle<ActionT>::SharedPtr shared_this,
    typename std::shared_ptr<const Feedback> feedback_message);

  /// Get a future to the goal result.
  /**
   * This method should not be called if the `ignore_result` flag was set when
   * sending the original goal request (see Client::async_send_goal).
   *
   * `is_result_aware()` can be used to check if it is safe to call this method.
   *
   * \throws exceptions::UnawareGoalHandleError If the the goal handle is unaware of the result.
   * \return A future to the result.
   */
  std::shared_future<WrappedResult>
  async_get_result();

  /// Returns the previous value of awareness
  bool
  set_result_awareness(bool awareness);

  void
  set_status(int8_t status);

  void
  set_result(const WrappedResult & wrapped_result);

  void
  invalidate(const exceptions::UnawareGoalHandleError & ex);

  bool
  is_invalidated() const;

  GoalInfo info_;

  std::exception_ptr invalidate_exception_{nullptr};

  bool is_result_aware_{false};
  std::promise<WrappedResult> result_promise_;
  std::shared_future<WrappedResult> result_future_;

  FeedbackCallback feedback_callback_{nullptr};
  ResultCallback result_callback_{nullptr};
  int8_t status_{GoalStatus::STATUS_ACCEPTED};

  rcpputils::PIMutex handle_mutex_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/client_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
