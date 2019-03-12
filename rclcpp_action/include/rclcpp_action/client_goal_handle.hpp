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

#include <rcl_action/action_client.h>

#include <action_msgs/msg/goal_status.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include <functional>
#include <future>
#include <memory>
#include <mutex>

#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

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

template<typename ActionT>
class ClientGoalHandle
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientGoalHandle)

  // A wrapper that defines the result of an action
  typedef struct WrappedResult
  {
    /// The unique identifier of the goal
    GoalUUID goal_id;
    /// A status to indicate if the goal was canceled, aborted, or suceeded
    ResultCode code;
    /// User defined fields sent back with an action
    typename ActionT::Result::SharedPtr result;
  } WrappedResult;

  using Feedback = typename ActionT::Feedback;
  using FeedbackCallback =
    std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;

  virtual ~ClientGoalHandle();

  const GoalUUID &
  get_goal_id() const;

  rclcpp::Time
  get_goal_stamp() const;

  std::shared_future<WrappedResult>
  async_result();

  int8_t
  get_status();

  bool
  is_feedback_aware();

  bool
  is_result_aware();

private:
  // The templated Client creates goal handles
  friend Client<ActionT>;

  ClientGoalHandle(const GoalInfo & info, FeedbackCallback callback);

  void
  set_feedback_callback(FeedbackCallback callback);

  void
  call_feedback_callback(
    typename ClientGoalHandle<ActionT>::SharedPtr shared_this,
    typename std::shared_ptr<const Feedback> feedback_message);

  void
  set_result_awareness(bool awareness);

  void
  set_status(int8_t status);

  void
  set_result(const WrappedResult & wrapped_result);

  void
  invalidate();

  GoalInfo info_;

  bool is_result_aware_{false};
  std::promise<WrappedResult> result_promise_;
  std::shared_future<WrappedResult> result_future_;

  FeedbackCallback feedback_callback_{nullptr};
  int8_t status_{GoalStatus::STATUS_ACCEPTED};

  std::mutex handle_mutex_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/client_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
