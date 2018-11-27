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

#include <functional>
#include <memory>
#include <string>

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
    const rosidl_action_type_support_t * type_support);

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
  CancelResponse
  call_handle_cancel_callback(rcl_action_goal_info_t &) = 0;

  /// Given a goal request message, return the UUID contained within.
  virtual
  std::array<uint8_t, 16>
  get_goal_id_from_goal_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  virtual
  std::shared_ptr<void>
  create_goal_request() = 0;

private:
  std::unique_ptr<ServerBaseImpl> pimpl_;
};

/// Templated Action Server class
/// It is responsible for getting the C action type support struct from the C++ type, and
/// calling user callbacks with goal handles of the appropriate type.
template<typename ACTION>
class Server : public ServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Server)

  using GoalCallback = std::function<GoalResponse (
    rcl_action_goal_info_t &, std::shared_ptr<typename ACTION::Goal>)>;
  using CancelCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ACTION>>)>;

  // TODO(sloretz) accept clock instance
  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::string & name,
    GoalCallback handle_goal,
    CancelCallback handle_cancel
  )
  : ServerBase(
      node_base,
      node_clock,
      name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>()),
    handle_goal_(handle_goal),
    handle_cancel_(handle_cancel)
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

    // TODO(sloretz) if goal is accepted then create a goal handle

    auto ros_response = std::make_shared<typename ACTION::GoalRequestService::Response>();
    ros_response->accepted = GoalResponse::ACCEPT == user_response;
    // TODO(sloretz) set timestamp in response and give that to ServerBase too
    return std::make_pair(user_response, ros_response);
  }

  CancelResponse
  call_handle_cancel_callback(rcl_action_goal_info_t & info)
  {
    // TODO(sloretz) look up goal handle and call users' callback with it
    (void)info;
    return CancelResponse::REJECT;
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

private:
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_HPP_
