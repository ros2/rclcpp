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

#include <rosidl_generator_c/action_type_support_struct.h>
#include <rosidl_typesupport_cpp/action_type_support.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/waitable.hpp>

#include <climits>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

namespace rclcpp_action
{
// Forward declaration
class ServerBaseImpl;

enum class GoalResponse : int8_t
{
  REJECT = 1,
  ACCEPT = 2,
};

enum class CancelResponse : int8_t
{
  REJECT = 1,
  ACCEPT = 2,
};

/// Base Action Server implementation
/// It is responsible for interfacing with the C action server API.
class ServerBase : public rclcpp::Waitable
{
public:
  // TODO(sloretz) NodeLoggingInterface when it can be gotten off a node
  RCLCPP_ACTION_PUBLIC
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::string & name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_server_options_t & options);

  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

  // -------------
  // Waitables API

  size_t
  get_number_of_ready_subscriptions() override;

  size_t
  get_number_of_ready_timers() override;

  size_t
  get_number_of_ready_clients() override;

  size_t
  get_number_of_ready_services() override;

  size_t
  get_number_of_ready_guard_conditions() override;

  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  bool
  is_ready(rcl_wait_set_t *) override;

  void
  execute() override;

  // End Waitables API
  // -----------------

protected:
  // ServerBase will call this function when a goal request is received.
  // The subclass should convert to the real type and call a user's callback.
  virtual
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback(rcl_action_goal_info_t &, std::shared_ptr<void> request) = 0;

  // ServerBase will determine which goal ids are being cancelled, and then call this function for
  // each goal id.
  // The subclass should look up a goal handle and call the user's callback.
  virtual
  std::pair<CancelResponse, std::shared_ptr<rcl_action_goal_handle_t>>
  call_handle_cancel_callback(const std::array<uint8_t, 16> & uuid) = 0;

  /// Given a goal request message, return the UUID contained within.
  virtual
  std::array<uint8_t, 16>
  get_goal_id_from_goal_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  virtual
  std::shared_ptr<void>
  create_goal_request() = 0;

  /// Call user callback to begin execution
  virtual
  void
  call_begin_execution_callback(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
    std::array<uint8_t, 16> uuid, std::shared_ptr<void> goal_request_message) = 0;

  /// Given a result request message, return the UUID contained within.
  virtual
  std::array<uint8_t, 16>
  get_goal_id_from_result_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  virtual
  std::shared_ptr<void>
  create_result_request() = 0;

  /// Create an empty goal result message so it can be sent as a reply in a lower layer
  virtual
  std::shared_ptr<void>
  create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) = 0;

  void
  publish_status();

  void
  publish_result(const std::array<uint8_t, 16> & uuid, std::shared_ptr<void> result_msg);

private:
  void
  execute_goal_request_received();

  void
  execute_cancel_request_received();

  void
  execute_result_request_received();

  void
  execute_check_expired_goals();

  std::unique_ptr<ServerBaseImpl> pimpl_;
};

struct UUIDHash
{
  size_t operator()(std::array<uint8_t, 16> const & uuid) const noexcept
  {
    // TODO(sloretz) Use someone else's hash function and cite it
    size_t result = 0;
    for (size_t i = 0; i < 16; ++i) {
      for (size_t b = 0; b < sizeof(size_t); ++b) {
        result ^= uuid[i] << CHAR_BIT * b;
      }
    }
    return result;
  }
};

/// Templated Action Server class
/// It is responsible for getting the C action type support struct from the C++ type, and
/// calling user callbacks with goal handles of the appropriate type.
template<typename ACTION>
class Server : public ServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Server)

  using GoalCallback = std::function<GoalResponse(
        rcl_action_goal_info_t &, std::shared_ptr<typename ACTION::Goal>)>;
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ACTION>>)>;
  using ExecuteCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ACTION>>)>;

  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::string & name,
    const rcl_action_server_options_t & options,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    ExecuteCallback handle_execute
  )
  : ServerBase(
      node_base,
      node_clock,
      name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>(),
      options),
    handle_goal_(handle_goal),
    handle_cancel_(handle_cancel),
    handle_execute_(handle_execute)
  {
  }

  virtual ~Server()
  {
  }

protected:
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback(rcl_action_goal_info_t & info, std::shared_ptr<void> message) override
  {
    // TODO(sloretz) update and remove assert when IDL pipeline allows nesting user's type
    static_assert(
      std::is_same<typename ACTION::Goal, typename ACTION::GoalRequestService::Request>::value,
      "Assuming user fields were merged with goal request fields");
    GoalResponse user_response = handle_goal_(
      info, std::static_pointer_cast<typename ACTION::Goal>(message));

    auto ros_response = std::make_shared<typename ACTION::GoalRequestService::Response>();
    ros_response->accepted = GoalResponse::ACCEPT == user_response;
    return std::make_pair(user_response, ros_response);
  }

  std::pair<CancelResponse, std::shared_ptr<rcl_action_goal_handle_t>>
  call_handle_cancel_callback(const std::array<uint8_t, 16> & uuid) override
  {
    CancelResponse resp = CancelResponse::REJECT;
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle;
    auto element = goal_handles_.find(uuid);
    if (element != goal_handles_.end()) {
      std::shared_ptr<ServerGoalHandle<ACTION>> goal_handle = element->second.lock();
      if (goal_handle) {
        resp = handle_cancel_(goal_handle);
        rcl_handle = goal_handle->get_rcl_handle();
      }
    }
    return std::make_pair(resp, rcl_handle);
  }

  void
  call_begin_execution_callback(
    std::shared_ptr<rcl_action_server_t> rcl_server,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
    std::array<uint8_t, 16> uuid, std::shared_ptr<void> goal_request_message) override
  {
    std::shared_ptr<ServerGoalHandle<ACTION>> goal_handle;
    // TODO(sloretz) how to make sure this lambda is not called beyond lifetime of this?
    std::function<void(const std::array<uint8_t, 16>&, std::shared_ptr<void>)> on_terminal_state =
      [this](const std::array<uint8_t, 16> & uuid, std::shared_ptr<void> result_message)
      {
        // Send result message to anyone that asked
        publish_result(uuid, result_message);
        // Publish a status message any time a goal handle changes state
        publish_status();
        // Delete data now (ServerBase and rcl_action_server_t keep data until goal handle expires)
        goal_handles_.erase(uuid);
      };
    goal_handle.reset(
      new ServerGoalHandle<ACTION>(
        rcl_server, rcl_goal_handle,
        uuid, std::static_pointer_cast<const typename ACTION::Goal>(goal_request_message),
        on_terminal_state));
    goal_handles_[uuid] = goal_handle;
    handle_execute_(goal_handle);
  }

  std::array<uint8_t, 16>
  get_goal_id_from_goal_request(void * message) override
  {
    return static_cast<typename ACTION::GoalRequestService::Request *>(message)->uuid;
  }

  std::shared_ptr<void>
  create_goal_request() override
  {
    return std::shared_ptr<void>(new typename ACTION::GoalRequestService::Request());
  }

  std::array<uint8_t, 16>
  get_goal_id_from_result_request(void * message) override
  {
    return static_cast<typename ACTION::GoalResultService::Request *>(message)->uuid;
  }

  std::shared_ptr<void>
  create_result_request() override
  {
    return std::shared_ptr<void>(new typename ACTION::GoalResultService::Request());
  }

  std::shared_ptr<void>
  create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) override
  {
    auto result = std::make_shared<typename ACTION::GoalResultService::Response>();
    result->status = status;
    return std::static_pointer_cast<void>(result);
  }

private:
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  ExecuteCallback handle_execute_;

  using GoalHandleWeakPtr = std::weak_ptr<ServerGoalHandle<ACTION>>;
  std::unordered_map<std::array<uint8_t, 16>, GoalHandleWeakPtr, UUIDHash> goal_handles_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_HPP_
