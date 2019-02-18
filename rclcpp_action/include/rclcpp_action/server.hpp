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

#ifndef RCLCPP_ACTION__SERVER_HPP_
#define RCLCPP_ACTION__SERVER_HPP_

#include <rcl_action/action_server.h>
#include <rosidl_generator_c/action_type_support_struct.h>
#include <rosidl_typesupport_cpp/action_type_support.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/waitable.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action
{
// Forward declaration
class ServerBaseImpl;

/// A response returned by an action server callback when a goal is requested.
enum class GoalResponse : int8_t
{
  /// The goal is rejected and will not be executed.
  REJECT = 1,
  /// The server accepts the goal, and is going to begin execution immediately.
  ACCEPT_AND_EXECUTE = 2,
  /// The server accepts the goal, and is going to execute it later.
  ACCEPT_AND_DEFER = 3,
};

/// A response returned by an action server callback when a goal has been asked to be canceled.
enum class CancelResponse : int8_t
{
  /// The server will not try to cancel the goal.
  REJECT = 1,
  /// The server has agreed to try to cancel the goal.
  ACCEPT = 2,
};

/// Base Action Server implementation
/// \internal
/**
 * This class should not be used directly by users writing an action server.
 * Instead users should use `rclcpp_action::Server<>`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ServerBase : public rclcpp::Waitable
{
public:
  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

  // -------------
  // Waitables API

  /// Return the number of subscriptions used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_subscriptions() override;

  /// Return the number of timers used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  /// Return the number of service clients used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  /// Return the number of service servers used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  /// Return the number of guard conditions used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  /// Add all entities to a wait set.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// Return true if any entity belonging to the action server is ready to be executed.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(rcl_wait_set_t *) override;

  /// Act on entities in the wait set which are ready to be acted upon.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute() override;

  // End Waitables API
  // -----------------

protected:
  RCLCPP_ACTION_PUBLIC
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_server_options_t & options);

  // -----------------------------------------------------
  // API for communication between ServerBase and Server<>

  // ServerBase will call this function when a goal request is received.
  // The subclass should convert to the real type and call a user's callback.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback(GoalUUID &, std::shared_ptr<void> request) = 0;

  // ServerBase will determine which goal ids are being cancelled, and then call this function for
  // each goal id.
  // The subclass should look up a goal handle and call the user's callback.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  CancelResponse
  call_handle_cancel_callback(const GoalUUID & uuid) = 0;

  /// Given a goal request message, return the UUID contained within.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  get_goal_id_from_goal_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_goal_request() = 0;

  /// Call user callback to inform them a goal has been accepted.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  call_goal_accepted_callback(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
    GoalUUID uuid, std::shared_ptr<void> goal_request_message) = 0;

  /// Given a result request message, return the UUID contained within.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  get_goal_id_from_result_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_result_request() = 0;

  /// Create an empty goal result message so it can be sent as a reply in a lower layer
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_status();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  notify_goal_terminal_state();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_feedback(std::shared_ptr<void> feedback_msg);

  // End API for communication between ServerBase and Server<>
  // ---------------------------------------------------------

private:
  /// Handle a request to add a new goal to the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_goal_request_received();

  /// Handle a request to cancel goals on the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_cancel_request_received();

  /// Handle a request to get the result of an action
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_result_request_received();

  /// Handle a timeout indicating a completed goal should be forgotten by the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_check_expired_goals();

  /// Private implementation
  /// \internal
  std::unique_ptr<ServerBaseImpl> pimpl_;
};

/// Action Server
/**
 * This class creates an action server.
 *
 * Create an instance of this server using `rclcpp_action::create_server()`.
 *
 * Internally, this class is responsible for:
 *  - coverting between the C++ action type and generic types for `rclcpp_action::ServerBase`, and
 *  - calling user callbacks.
 */
template<typename ActionT>
class Server : public ServerBase, public std::enable_shared_from_this<Server<ActionT>>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Server)

  /// Signature of a callback that accepts or rejects goal requests.
  using GoalCallback = std::function<GoalResponse(
        const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
  /// Signature of a callback that accepts or rejects requests to cancel a goal.
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  /// Signature of a callback that is used to notify when the goal has been accepted.
  using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;

  /// Construct an action server.
  /**
   * This constructs an action server, but it will not work until it has been added to a node.
   * Use `rclcpp_action::create_server()` to both construct and add to a node.
   *
   * Three callbacks must be provided:
   *  - one to accept or reject goals sent to the server,
   *  - one to accept or reject requests to cancel a goal,
   *  - one to receive a goal handle after a goal has been accepted.
   * All callbacks must be non-blocking.
   * The result of a goal should be set using methods on `rclcpp_action::ServerGoalHandle<>`.
   *
   * \param[in] node_base a pointer to the base interface of a node.
   * \param[in] node_clock a pointer to an interface that allows getting a node's clock.
   * \param[in] node_logging a pointer to an interface that allows getting a node's logger.
   * \param[in] name the name of an action.
   *  The same name and type must be used by both the action client and action server to
   *  communicate.
   * \param[in] options options to pass to the underlying `rcl_action_server_t`.
   * \param[in] handle_goal a callback that decides if a goal should be accepted or rejected.
   * \param[in] handle_cancel a callback that decides if a goal should be attemted to be canceled.
   *  The return from this callback only indicates if the server will try to cancel a goal.
   *  It does not indicate if the goal was actually canceled.
   * \param[in] handle_accepted a callback that is called to give the user a handle to the goal.
   *  execution.
   */
  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rcl_action_server_options_t & options,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted
  )
  : ServerBase(
      node_base,
      node_clock,
      node_logging,
      name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
      options),
    handle_goal_(handle_goal),
    handle_cancel_(handle_cancel),
    handle_accepted_(handle_accepted)
  {
  }

  virtual ~Server() = default;

protected:
  // -----------------------------------------------------
  // API for communication between ServerBase and Server<>

  /// \internal
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback(GoalUUID & uuid, std::shared_ptr<void> message) override
  {
    auto request = std::static_pointer_cast<
      typename ActionT::Impl::SendGoalService::Request>(message);
    auto goal = std::shared_ptr<typename ActionT::Goal>(request, &request->goal);
    GoalResponse user_response = handle_goal_(uuid, goal);

    auto ros_response = std::make_shared<typename ActionT::Impl::SendGoalService::Response>();
    ros_response->accepted = GoalResponse::ACCEPT_AND_EXECUTE == user_response ||
      GoalResponse::ACCEPT_AND_DEFER == user_response;
    return std::make_pair(user_response, ros_response);
  }

  /// \internal
  CancelResponse
  call_handle_cancel_callback(const GoalUUID & uuid) override
  {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    CancelResponse resp = CancelResponse::REJECT;
    auto element = goal_handles_.find(uuid);
    if (element != goal_handles_.end()) {
      std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle = element->second.lock();
      if (goal_handle) {
        resp = handle_cancel_(goal_handle);
        if (CancelResponse::ACCEPT == resp) {
          goal_handle->_set_canceling();
        }
      }
    }
    return resp;
  }

  /// \internal
  void
  call_goal_accepted_callback(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
    GoalUUID uuid, std::shared_ptr<void> goal_request_message) override
  {
    std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle;
    std::weak_ptr<Server<ActionT>> weak_this = this->shared_from_this();

    std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state =
      [weak_this](const GoalUUID & uuid, std::shared_ptr<void> result_message)
      {
        std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        // Send result message to anyone that asked
        shared_this->publish_result(uuid, result_message);
        // Publish a status message any time a goal handle changes state
        shared_this->publish_status();
        // notify base so it can recalculate the expired goal timer
        shared_this->notify_goal_terminal_state();
        // Delete data now (ServerBase and rcl_action_server_t keep data until goal handle expires)
        std::lock_guard<std::mutex> lock(shared_this->goal_handles_mutex_);
        shared_this->goal_handles_.erase(uuid);
      };

    std::function<void(const GoalUUID &)> on_executing =
      [weak_this](const GoalUUID & uuid)
      {
        std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        (void)uuid;
        // Publish a status message any time a goal handle changes state
        shared_this->publish_status();
      };

    std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)> publish_feedback =
      [weak_this](std::shared_ptr<typename ActionT::Impl::FeedbackMessage> feedback_msg)
      {
        std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
        if (!shared_this) {
          return;
        }
        shared_this->publish_feedback(std::static_pointer_cast<void>(feedback_msg));
      };

    auto request = std::static_pointer_cast<
      const typename ActionT::Impl::SendGoalService::Request>(goal_request_message);
    auto goal = std::shared_ptr<const typename ActionT::Goal>(request, &request->goal);
    goal_handle.reset(
      new ServerGoalHandle<ActionT>(
        rcl_goal_handle, uuid, goal, on_terminal_state, on_executing, publish_feedback));
    {
      std::lock_guard<std::mutex> lock(goal_handles_mutex_);
      goal_handles_[uuid] = goal_handle;
    }
    handle_accepted_(goal_handle);
  }

  /// \internal
  GoalUUID
  get_goal_id_from_goal_request(void * message) override
  {
    return
      static_cast<typename ActionT::Impl::SendGoalService::Request *>(message)->goal_id.uuid;
  }

  /// \internal
  std::shared_ptr<void>
  create_goal_request() override
  {
    return std::shared_ptr<void>(new typename ActionT::Impl::SendGoalService::Request());
  }

  /// \internal
  GoalUUID
  get_goal_id_from_result_request(void * message) override
  {
    return
      static_cast<typename ActionT::Impl::GetResultService::Request *>(message)->goal_id.uuid;
  }

  /// \internal
  std::shared_ptr<void>
  create_result_request() override
  {
    return std::shared_ptr<void>(new typename ActionT::Impl::GetResultService::Request());
  }

  /// \internal
  std::shared_ptr<void>
  create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) override
  {
    auto result = std::make_shared<typename ActionT::Impl::GetResultService::Response>();
    result->status = status;
    return std::static_pointer_cast<void>(result);
  }

  // End API for communication between ServerBase and Server<>
  // ---------------------------------------------------------

private:
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  AcceptedCallback handle_accepted_;

  using GoalHandleWeakPtr = std::weak_ptr<ServerGoalHandle<ActionT>>;
  /// A map of goal id to goal handle weak pointers.
  /// This is used to provide a goal handle to handle_cancel.
  std::unordered_map<GoalUUID, GoalHandleWeakPtr> goal_handles_;
  std::mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_HPP_
