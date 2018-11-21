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
class ClientBase
{
public:
  // TODO(sloretz) NodeLoggingInterface when it can be gotten off a node
  RCLCPP_ACTION_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & name,
    const rosidl_action_type_support_t * type_support);

  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  // TODO(sloretz) add a link between this class and callbacks in the templated

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

  using FeedbackCallback = std::function<void (
        std::shared_ptr<ClientGoalHandle<ACTION>>, const typename ACTION::Feedback)>;

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & name
  )
  : ClientBase(
      node_base,
      name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>())
  {
    // TODO(sloretz) what's the link that causes a feedback topic to be called ?
  }

  ClientGoalHandle<ACTION>
  async_send_goal(typename ACTION::Goal * goal, FeedbackCallback callback = nullptr);

  RCLCPP_ACTION_PUBLIC
  std::future<typename ACTION::CancelGoalService::Response>
  async_cancel_all_goals();

  RCLCPP_ACTION_PUBLIC
  std::future<typename ACTION::CancelGoalService::Response>
  async_cancel_goals_before(rclcpp::Time);

  virtual ~Client()
  {
  }
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CLIENT_HPP_
