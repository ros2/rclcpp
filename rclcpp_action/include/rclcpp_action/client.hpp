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

#include <rclcpp/macros.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/waitable.hpp>

#include <rosidl_generator_c/action_type_support_struct.h>
#include <rosidl_typesupport_cpp/action_type_support.hpp>

#include <algorithm>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"


namespace rclcpp_action
{
// Forward declaration
class ClientBaseImpl;

/// Base Action Client implementation
/// It is responsible for interfacing with the C action client API.
class ClientBase : public rclcpp::Waitable
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
  size_t
  get_number_of_ready_subscriptions() override;

  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  RCLCPP_ACTION_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

  RCLCPP_ACTION_PUBLIC
  void
  execute() override;

protected:
  using ResponseCallback =
    std::function<void(std::shared_ptr<void> response)>;

  RCLCPP_ACTION_PUBLIC
  rclcpp::Logger get_logger();

  RCLCPP_ACTION_PUBLIC
  virtual GoalID generate_goal_id();

  RCLCPP_ACTION_PUBLIC
  virtual void send_goal_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  RCLCPP_ACTION_PUBLIC
  virtual void send_result_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  RCLCPP_ACTION_PUBLIC
  virtual void send_cancel_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

private:
  virtual std::shared_ptr<void> create_goal_response() const = 0;

  virtual void handle_goal_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> goal_response);

  virtual std::shared_ptr<void> create_result_response() const = 0;

  virtual void handle_result_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> result_response);

  virtual std::shared_ptr<void> create_cancel_response() const = 0;

  virtual void handle_cancel_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> cancel_response);

  virtual std::shared_ptr<void> create_feedback_message() const = 0;

  virtual void handle_feedback_message(std::shared_ptr<void> message) = 0;

  virtual std::shared_ptr<void> create_status_message() const = 0;

  virtual void handle_status_message(std::shared_ptr<void> message) = 0;

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
  using GoalHandle = ClientGoalHandle<ACTION>;
  using Result = typename GoalHandle::Result;
  using FeedbackCallback = std::function<void (
      typename GoalHandle::SharedPtr, const Feedback &)>;
  using CancelRequest = typename ACTION::CancelGoalService::Request;
  using CancelResponse = typename ACTION::CancelGoalService::Response;

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & action_name
  )
  : Client(node_base, action_name, rcl_action_client_get_default_options())
  {
  }

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
  std::shared_future<typename GoalHandle::SharedPtr> async_send_goal(
    const Goal & goal, FeedbackCallback callback = nullptr, bool ignore_result = false)
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename GoalHandle::SharedPtr>>();
    std::shared_future<typename GoalHandle::SharedPtr> future(promise->get_future());
    using GoalRequest = typename ACTION::GoalRequestService::Request;
    // auto goal_request = std::make_shared<GoalRequest>();
    // goal_request->goal_id = this->generate_goal_id();
    // goal_request->goal = goal;
    auto goal_request = std::make_shared<GoalRequest>(goal);
    goal_request->uuid = this->generate_goal_id();
    this->send_goal_request(
      std::static_pointer_cast<void>(goal_request),
      [this, goal_request, callback, ignore_result,
       promise] (std::shared_ptr<void> response) mutable
      {
        using GoalResponse = typename ACTION::GoalRequestService::Response;
        typename GoalResponse::SharedPtr goal_response =
          std::static_pointer_cast<GoalResponse>(response);
        if (!goal_response->accepted) {
          promise->set_exception(std::make_exception_ptr(exceptions::RejectedGoalError()));
          return;
        }
        GoalInfo goal_info;
        // goal_info.goal_id = goal_request->goal_id;
        goal_info.goal_id.uuid = goal_request->uuid;
        goal_info.stamp = goal_response->stamp;
        // Do not use std::make_shared as friendship cannot be forwarded.
        std::shared_ptr<GoalHandle> goal_handle(new GoalHandle(goal_info, callback));
        if (!ignore_result) {
          try {
            this->make_result_aware(goal_handle);
          } catch (...) {
            promise->set_exception(std::current_exception());
            return;
          }
        }
        std::lock_guard<std::mutex> guard(goal_handles_mutex_);
        goal_handles_[goal_handle->get_goal_id()] = goal_handle;
        promise->set_value(goal_handle);
      });
    return future;
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<Result>
  async_get_result(typename GoalHandle::SharedPtr goal_handle) {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError(goal_handle);
    }
    if (!goal_handle->is_result_aware()) {
      this->make_result_aware(goal_handle);
    }
    return goal_handle->async_result();
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<bool>
  async_cancel_goal(typename GoalHandle::SharedPtr goal_handle) {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError();
    }
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<bool>>();
    std::shared_future<bool> future(promise->get_future());
    auto cancel_request = std::make_shared<CancelRequest>();
    // cancel_request->goal_info.goal_id = goal_handle->get_goal_id();
    cancel_request->goal_info.goal_id.uuid = goal_handle->get_goal_id();
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [goal_handle, promise] (std::shared_ptr<void> response) mutable
      {
        typename CancelResponse::SharedPtr cancel_response =
          std::static_pointer_cast<CancelResponse>(response);
        bool goal_canceled = false;
        if (!cancel_response->goals_canceling.empty()) {
          const GoalInfo & canceled_goal_info = cancel_response->goals_canceling[0];
          // goal_canceled = (canceled_goal_info.goal_id == goal_handle->get_goal_id());
          goal_canceled = (canceled_goal_info.goal_id.uuid == goal_handle->get_goal_id());
        }
        promise->set_value(goal_canceled);
      });
    return future;
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_all_goals()
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(cancel_request->goal_info.uuid, 0u);
    return async_cancel(cancel_request);
  }

  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goals_before(const rclcpp::Time & stamp)
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(cancel_request->goal_info.uuid, 0u);
    cancel_request->goal_info.stamp = stamp;
    return async_cancel(cancel_request);
  }

  virtual ~Client()
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.begin();
    while (it != goal_handles_.end()) {
      it->second->invalidate();
      it = goal_handles_.erase(it);
    }
  }

private:
  std::shared_ptr<void> create_goal_response() const override
  {
    using GoalResponse = typename ACTION::GoalRequestService::Response;
    return std::shared_ptr<void>(new GoalResponse());
  }

  std::shared_ptr<void> create_result_response() const override
  {
    using GoalResultResponse = typename ACTION::GoalResultService::Response;
    return std::shared_ptr<void>(new GoalResultResponse());
  }

  std::shared_ptr<void> create_cancel_response() const override
  {
    return std::shared_ptr<void>(new CancelResponse());
  }

  std::shared_ptr<void> create_feedback_message() const override
  {
    // using FeedbackMessage = typename ACTION::FeedbackMessage;
    // return std::shared_ptr<void>(new FeedbackMessage());
    return std::shared_ptr<void>(new Feedback());
  }

  void handle_feedback_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    // using FeedbackMessage = typename ACTION::FeedbackMessage;
    // typename FeedbackMessage::SharedPtr feedback_message =
    //   std::static_pointer_cast<FeedbackMessage>(message);
    typename Feedback::SharedPtr feedback_message =
      std::static_pointer_cast<Feedback>(message);
    // const GoalID & goal_id = feedback_message->goal_id;
    const GoalID & goal_id = feedback_message->uuid;
    if (goal_handles_.count(goal_id) == 0) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received feedback for unknown goal. Ignoring...");
      return;
    }
    typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id];
    if (!goal_handle->is_feedback_aware()) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received feedback for goal but it ignores it.");
      return;
    }
    const FeedbackCallback & callback = goal_handle->get_feedback_callback();
    // callback(goal_handle, feedback_message->feedback);
    callback(goal_handle, *feedback_message);
  }

  std::shared_ptr<void> create_status_message() const override
  {
    using GoalStatusMessage = typename ACTION::GoalStatusMessage;
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  void handle_status_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    using GoalStatusMessage = typename ACTION::GoalStatusMessage;
    typename GoalStatusMessage::SharedPtr status_message =
        std::static_pointer_cast<GoalStatusMessage>(message);
    for (const GoalStatus & status : status_message->status_list) {
      // const GoalID & goal_id = status.goal_info.goal_id;
      const GoalID & goal_id = status.goal_info.goal_id.uuid;
      if (goal_handles_.count(goal_id) == 0) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Received status for unknown goal. Ignoring...");
        continue;
      }
      typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id];
      goal_handle->set_status(status.status);
      const int8_t goal_status = goal_handle->get_status();
      if (
        goal_status == GoalStatus::STATUS_SUCCEEDED ||
        goal_status == GoalStatus::STATUS_CANCELED ||
        goal_status == GoalStatus::STATUS_ABORTED
      ) {
        goal_handles_.erase(goal_id);
      }
    }
  }

  void make_result_aware(typename GoalHandle::SharedPtr goal_handle) {
    using GoalResultRequest = typename ACTION::GoalResultService::Request;
    auto goal_result_request = std::make_shared<GoalResultRequest>();
    // goal_result_request.goal_id = goal_handle->get_goal_id();
    goal_result_request->uuid = goal_handle->get_goal_id();
    this->send_result_request(
      std::static_pointer_cast<void>(goal_result_request),
      [goal_handle, this] (std::shared_ptr<void> response) mutable
      {
        // Wrap the response in a struct with the fields a user cares about
        Result result;
        using GoalResultResponse = typename ACTION::GoalResultService::Response;
        result.response = std::static_pointer_cast<GoalResultResponse>(response);
        result.goal_id = goal_handle->get_goal_id();
        result.code = static_cast<ResultCode>(result.response->status);
        goal_handle->set_result(result);
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        goal_handles_.erase(goal_handle->get_goal_id());
      });
    goal_handle->set_result_awareness(true);
  }

  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel(typename CancelRequest::SharedPtr cancel_request)
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename CancelResponse::SharedPtr>>();
    std::shared_future<typename CancelResponse::SharedPtr> future(promise->get_future());
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [promise] (std::shared_ptr<void> response) mutable
      {
        typename CancelResponse::SharedPtr cancel_response =
          std::static_pointer_cast<CancelResponse>(response);
        promise->set_value(cancel_response);
      });
    return future;
  }

  std::map<GoalID, typename GoalHandle::SharedPtr> goal_handles_;
  std::mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_HPP_
