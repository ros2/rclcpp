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

#ifndef RCLCPP_ACTION__CLIENT_HPP_
#define RCLCPP_ACTION__CLIENT_HPP_


#include <rosidl_generator_c/action_type_support_struct.h>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/time.hpp>
#include <rosidl_typesupport_cpp/action_type_support.hpp>

#include <functional>
#include <memory>
#include <string>

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/visibility_control.hpp"


namespace rclcpp_action
{
// Forward declaration
class ClientBaseImpl;

/// Base Action Client implementation
/// It is responsible for interfacing with the C action client API.
class ClientBase : public Waitable
{
public:
  // TODO(sloretz) NodeLoggingInterface when it can be gotten off a node
  RCLCPP_ACTION_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & options);

  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  RCLCPP_ACTION_PUBLIC
  void handle_goal_response(
    std::shared_ptr<rmw_request_id_t> response_header,
    std::shared_ptr<void> response);

  RCLCPP_ACTION_PUBLIC
  void handle_result_response(
    std::shared_ptr<rmw_request_id_t> response_header,
    std::shared_ptr<void> response);

  RCLCPP_ACTION_PUBLIC
  void handle_cancel_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response);

protected:
  RCLCPP_ACTION_PUBLIC  
  void send_goal_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  RCLCPP_ACTION_PUBLIC
  void send_result_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  RCLCPP_ACTION_PUBLIC  
  void send_cancel_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

private:
  std::unique_ptr<ClientBaseImpl> pimpl_;
};


/// Templated Action Client class
/// It is responsible for getting the C action type support struct from the C++ type, and
/// calling user callbacks with goal handles of the appropriate type.
template<typename ACTION>
class Client : public ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Client)

  using Goal = typename ACTION::Goal;
  using Feedback = typename ACTION::Feedback;
  using Result = typename ACTION::Result;
  using GoalHandle = ClientGoalHandle<ACTION>;
  using FeedbackCallback = std::function<void (
      GoalHandle::SharedPtr, const Feedback &)>;
  using CancelRequest = typename ACTION::CancelGoalService::Request;
  using CancelResponse = typename ACTION::CancelGoalService::Response;
  
  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & action_name, const rcl_action_client_options_t & client_options
  )
  : ClientBase(
      node_base, action_name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>(),
      client_options)
  {
  }

  RCLCPP_ACTION_PUBLIC
  void handle_feedback(std::shared_ptr<void> message) {
    using Feedback = typename ACTION::Feedback;
    Feedback::SharedPtr feedback = std::static_pointer_cast<Feedback>(message);
    std::unique_lock<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(feedback->goal_id) == 0) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rclcpp", "Received feedback for unknown goal. Ignoring...");
      return;
    }
    GoalHandle::SharedPtr goal_handle = goal_handles_[feedback->goal_id];
    if (!goal_handle->is_feedback_aware()) {
      RCUTILS_LOG_DEBUG_NAMED(
        "rclcpp", "Received feedback for goal %s, but it ignores feedback.");
      return;
    }
    const FeedbackCallback & callback = goal_handle->get_feedback_callback();
    callback(goal_handle, *feedback);
  }

  RCLCPP_ACTION_PUBLIC
  void handle_status(std::shared_ptr<void> message)
  {
    using GoalStatusArray = typename ACTION::GoalStatusArray;
    GoalStatusArray::SharedPtr status_array =
        std::static_pointer_cast<GoalStatusArray>(message);
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    for (const GoalStatus & status : status_array.status_list)
    {
      const GoalID & goal_id = status.goal_info.goal_id;
      if (goal_handles_.count(goal_id) != 0)
      {
        goal_handles_[goal_id]->set_status(status);
        if (status.status == STATUS_CANCELED || status.status == STATUS_ABORTED)
        {
          goal_handles_.erase(goal_id);
        }
      }
    }
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<GoalHandle::SharedPtr> async_send_goal(
    Goal::SharedPtr goal, FeedbackCallback callback = nullptr, bool ignore_result = false)
  {
    
    if (pending_goal_handles_.count(goal.goal_id) != 0) {

    }
    if (goal_handles_.count(goal.goal_id) != 0) {

    }
    std::promise<GoalHandle::SharedPtr> promise;
    std::shared_future<GoalHandle::SharedPtr> future(promise.get_future());
    this->send_goal_request(
      std::static_pointer_cast<void>(goal),
      [this, promise{std::move(promise)}, callback] (std::shared_ptr<void> response)
      {
        using GoalResponse = typename ACTION::GoalService::Response;
        GoalResponse::SharedPtr goal_response =
          std::static_pointer_cast<GoalResponse>(response);
        if (!goal_response.accepted) {
          promise->set_exception(
            std::make_exception_ptr(RejectedGoalError("")));
        }
        auto goal_handle = std::make_shared<GoalHandle>(goal_, callback);
        if (!ignore_result) {
          try {
            this->async_get_result(goal_handle);
          } catch (...) {
            promise->set_exception(std::current_exception());
            return;
          }
        }
        goal_handles_[goal_id] = goal_handles;
        pending_goal_handles_.erase(goal_id);
        promise->set_value(goal_handle);
      });
    return future;
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<Result::SharedPtr>
  async_get_result_response(GoalHandle::SharedPtr goal_handle) {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    const GoalID & goal_id = goal_handle->goal_id();
    if (goal_handles_.count(goal_id) == 0) {
      throw;
    }
    if (!goal_handle->tracks_result()) {
      using ResultRequest = typename ACTION::GoalResultService::Request;
      auto result_request = std::make_shared<ResultRequest>();
      result_request.goal_id = goal_handle->goal_id();
      this->send_result_request(
        std::static_pointer_cast<void>(result_request),
        [goal_handle] (std::shared_ptr<void> response) {
          Result::SharedPtr result =
              std::static_pointer_cast<Result>(response);
          goal_handle->set_result(result);
        });
      goal_handle->set_result_tracking();
    }
    return goal_handle->async_result();
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<bool>
  async_cancel_goal(GoalHandle::SharedPtr goal_handle) {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    const GoalID & goal_id = goal_handle->goal_id();
    if (goal_handles_.count(goal_id) == 0) {
      throw;
    }
    std::promise<bool> promise;
    std::shared_future<bool> future(promise.get_future());
    auto cancel_request = std::make_shared<CancelRequest>();
    cancel_request->goal_info.goal_id = goal_id;
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [this, goal_handle, promise{std::move(promise)}] (std::shared_ptr<void> response) {
        CancelGoalResponse::SharedPtr cancel_response =
          std::static_pointer_cast<CancelGoalResponse>(response);
        bool goal_canceled = false;
        if (!cancel_response->goals_canceling.empty()) {
          const GoalInfo & canceled_goal_info = cancel_response->goals_canceling[0];
          const GoalID & canceled_goal_id = canceled_goal_info.goal_id;
          const GoalID & to_be_canceled_goal_id = goal_handle->goal_id();
          if (!uuidcmp(canceled_goal_id.uuid, to_be_canceled_goal_id.uuid)) {
            goal_canceled = canceled_goal_info.accepted;
          }
        }
        promise.set_value(goal_canceled);
      });
    return future;
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<CancelResponse::SharedPtr>
  async_cancel_all_goals() {
    auto cancel_request = std::make_shared<CancelRequest>();
    cancel_request->goal_info.goal_id.uuid = zerouuid;
    return async_cancel(cancel_request);
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<CancelResponse::SharedPtr>
  async_cancel_goals_before(const rclcpp::Time & stamp) {
    auto cancel_request = std::make_shared<CancelRequest>();
    cancel_request->goal_info.goal_id.uuid = zerouuid;
    cancel_request->goal_info.stamp = stamp;
    return async_cancel(cancel_request);
  }

  virtual ~Client()
  {
    std::unique_lock<std::mutex> lock(goal_handles_mutex_);
    auto it = goal_handles_.begin();
    while (it != it->goal_handles_.end()) {
      it->second->set_invalid();
      it = goal_handles_.erase(it);
    }
  }

 private:
  std::shared_future<CancelGoalResponse::SharedPtr>
  async_cancel(CancelGoalRequest::SharedPtr cancel_request) {
    std::promise<CancelGoalResponse::SharedPtr> promise;
    std::shared_future<CancelGoalResponse::SharedPtr> future(promise.get_future());
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [promise{std::move(promise)}] (std::shared_ptr<void> response) {
        CancelGoalResponse::SharedPtr cancel_response =
            std::static_pointer_cast<CancelGoalResponse>(response);
        promise.set_value(cancel_response);
      });
    return future;
  }

  std::map<GoalID, GoalHandle::SharedPtr> goal_handles_;
  std::mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_HPP_
