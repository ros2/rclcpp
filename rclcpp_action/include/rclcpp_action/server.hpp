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

#include <functional>
#include <memory>
#include <string>

#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

namespace rclcpp_action
{
// Forward declaration
class ServerBaseImpl;

/// Base Action Server implementation
/// It is responsible for interfacing with the C action server API.
class ServerBase
{
public:
  // TODO(sloretz) NodeLoggingInterface when it can be gotten off a node
  // TODO(sloretz) accept clock instance
  RCLCPP_ACTION_PUBLIC
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & name,
    const rosidl_action_type_support_t * type_support);

  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

  // TODO(sloretz) add a link between this class and callbacks in the server class

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

  using GoalCallback = std::function<void (rcl_action_goal_info_t &, typename ACTION::Goal *)>;
  using CancelCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ACTION>>)>;

  // TODO(sloretz) accept clock instance
  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & name,
    GoalCallback handle_goal,
    CancelCallback handle_cancel
  )
  : ServerBase(
      node_base,
      name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>()),
    handle_goal_(handle_goal),
    handle_cancel_(handle_cancel)
  {
    // TODO(sloretz) what's the link that causes `handle_goal_` and `handle_cancel_` to be called?
  }

  virtual ~Server()
  {
  }

private:
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_HPP_
