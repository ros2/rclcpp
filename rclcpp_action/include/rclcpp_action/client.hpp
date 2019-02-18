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
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/waitable.hpp>

#include <rosidl_generator_c/action_type_support_struct.h>
#include <rosidl_typesupport_cpp/action_type_support.hpp>

#include <algorithm>
#include <chrono>
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
/// \internal
/**
 * This class should not be used directly by users wanting to create an aciton client.
 * Instead users should use `rclcpp_action::Client<>`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ClientBase : public rclcpp::Waitable
{
public:
  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  /// Return true if there is an action server that is ready to take goal requests.
  RCLCPP_ACTION_PUBLIC
  bool
  action_server_is_ready() const;

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  template<typename RatioT = std::milli>
  bool
  wait_for_action_server(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return wait_for_action_server_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  // -------------
  // Waitables API

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_subscriptions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute() override;

  // End Waitables API
  // -----------------

protected:
  RCLCPP_ACTION_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & options);

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  RCLCPP_ACTION_PUBLIC
  bool
  wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout);

  // -----------------------------------------------------
  // API for communication between ClientBase and Client<>
  using ResponseCallback = std::function<void (std::shared_ptr<void> response)>;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  rclcpp::Logger get_logger();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  generate_goal_id();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_goal_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_result_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_cancel_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_goal_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_goal_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> goal_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_result_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_result_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> result_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_cancel_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_cancel_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> cancel_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_feedback_message() const = 0;

  /// \internal
  virtual
  void
  handle_feedback_message(std::shared_ptr<void> message) = 0;

  /// \internal
  virtual
  std::shared_ptr<void>
  create_status_message() const = 0;

  /// \internal
  virtual
  void
  handle_status_message(std::shared_ptr<void> message) = 0;

  // End API for communication between ClientBase and Client<>
  // ---------------------------------------------------------

private:
  std::unique_ptr<ClientBaseImpl> pimpl_;
};

/// Action Client
/**
 * This class creates an action client.
 *
 * Create an instance of this server using `rclcpp_action::create_client()`.
 *
 * Internally, this class is responsible for:
 *  - coverting between the C++ action type and generic types for `rclcpp_action::ClientBase`, and
 *  - calling user callbacks.
 */
template<typename ActionT>
class Client : public ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Client<ActionT>)

  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using FeedbackCallback = typename ClientGoalHandle<ActionT>::FeedbackCallback;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rcl_action_client_options_t client_options = rcl_action_client_get_default_options()
  )
  : ClientBase(
      node_base, node_graph, node_logging, action_name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
      client_options)
  {
  }

  std::shared_future<typename GoalHandle::SharedPtr>
  async_send_goal(
    const Goal & goal, FeedbackCallback callback = nullptr, bool ignore_result = false)
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename GoalHandle::SharedPtr>>();
    std::shared_future<typename GoalHandle::SharedPtr> future(promise->get_future());
    using GoalRequest = typename ActionT::Impl::SendGoalService::Request;
    auto goal_request = std::make_shared<GoalRequest>();
    goal_request->goal_id.uuid = this->generate_goal_id();
    goal_request->goal = goal;
    this->send_goal_request(
      std::static_pointer_cast<void>(goal_request),
      [this, goal_request, callback, ignore_result, promise](
        std::shared_ptr<void> response) mutable
      {
        using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
        auto goal_response = std::static_pointer_cast<GoalResponse>(response);
        if (!goal_response->accepted) {
          promise->set_value(nullptr);
          return;
        }
        GoalInfo goal_info;
        goal_info.goal_id.uuid = goal_request->goal_id.uuid;
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

  std::shared_future<WrappedResult>
  async_get_result(typename GoalHandle::SharedPtr goal_handle)
  {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError();
    }
    // If the user chose to ignore the result before, then ask the server for the result now.
    if (!goal_handle->is_result_aware()) {
      this->make_result_aware(goal_handle);
    }
    return goal_handle->async_result();
  }

  std::shared_future<bool>
  async_cancel_goal(typename GoalHandle::SharedPtr goal_handle)
  {
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
      [goal_handle, promise](std::shared_ptr<void> response) mutable
      {
        auto cancel_response = std::static_pointer_cast<CancelResponse>(response);
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

  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_all_goals()
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(
      cancel_request->goal_info.goal_id.uuid.begin(),
      cancel_request->goal_info.goal_id.uuid.end(), 0u);
    return async_cancel(cancel_request);
  }

  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goals_before(const rclcpp::Time & stamp)
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(
      cancel_request->goal_info.goal_id.uuid.begin(),
      cancel_request->goal_info.goal_id.uuid.end(), 0u);
    cancel_request->goal_info.stamp = stamp;
    return async_cancel(cancel_request);
  }

  virtual
  ~Client()
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.begin();
    while (it != goal_handles_.end()) {
      it->second->invalidate();
      it = goal_handles_.erase(it);
    }
  }

private:
  /// \internal
  std::shared_ptr<void>
  create_goal_response() const override
  {
    using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
    return std::shared_ptr<void>(new GoalResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_result_response() const override
  {
    using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
    return std::shared_ptr<void>(new GoalResultResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_cancel_response() const override
  {
    return std::shared_ptr<void>(new CancelResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_feedback_message() const override
  {
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    return std::shared_ptr<void>(new FeedbackMessage());
  }

  /// \internal
  void
  handle_feedback_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    typename FeedbackMessage::SharedPtr feedback_message =
      std::static_pointer_cast<FeedbackMessage>(message);
    const GoalUUID & goal_id = feedback_message->goal_id.uuid;
    if (goal_handles_.count(goal_id) == 0) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received feedback for unknown goal. Ignoring...");
      return;
    }
    typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id];
    auto feedback = std::make_shared<Feedback>();
    *feedback = feedback_message->feedback;
    goal_handle->call_feedback_callback(goal_handle, feedback);
  }

  /// \internal
  std::shared_ptr<void>
  create_status_message() const override
  {
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  /// \internal
  void
  handle_status_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;
    auto status_message = std::static_pointer_cast<GoalStatusMessage>(message);
    for (const GoalStatus & status : status_message->status_list) {
      // const GoalUUID & goal_id = status.goal_info.goal_id;
      const GoalUUID & goal_id = status.goal_info.goal_id.uuid;
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
        goal_status == GoalStatus::STATUS_ABORTED)
      {
        goal_handles_.erase(goal_id);
      }
    }
  }

  /// \internal
  void
  make_result_aware(typename GoalHandle::SharedPtr goal_handle)
  {
    using GoalResultRequest = typename ActionT::Impl::GetResultService::Request;
    auto goal_result_request = std::make_shared<GoalResultRequest>();
    // goal_result_request.goal_id = goal_handle->get_goal_id();
    goal_result_request->goal_id.uuid = goal_handle->get_goal_id();
    this->send_result_request(
      std::static_pointer_cast<void>(goal_result_request),
      [goal_handle, this](std::shared_ptr<void> response) mutable
      {
        // Wrap the response in a struct with the fields a user cares about
        WrappedResult wrapped_result;
        using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
        auto result_response = std::static_pointer_cast<GoalResultResponse>(response);
        wrapped_result.result = std::make_shared<typename ActionT::Result>();
        *wrapped_result.result = result_response->result;
        wrapped_result.goal_id = goal_handle->get_goal_id();
        wrapped_result.code = static_cast<ResultCode>(result_response->status);
        goal_handle->set_result(wrapped_result);
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        goal_handles_.erase(goal_handle->get_goal_id());
      });
    goal_handle->set_result_awareness(true);
  }

  /// \internal
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel(typename CancelRequest::SharedPtr cancel_request)
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename CancelResponse::SharedPtr>>();
    std::shared_future<typename CancelResponse::SharedPtr> future(promise->get_future());
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [promise](std::shared_ptr<void> response) mutable
      {
        auto cancel_response = std::static_pointer_cast<CancelResponse>(response);
        promise->set_value(cancel_response);
      });
    return future;
  }

  std::map<GoalUUID, typename GoalHandle::SharedPtr> goal_handles_;
  std::mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_HPP_
