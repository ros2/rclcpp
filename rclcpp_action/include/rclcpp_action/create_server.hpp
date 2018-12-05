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

#ifndef RCLCPP_ACTION__CREATE_SERVER_HPP_
#define RCLCPP_ACTION__CREATE_SERVER_HPP_

#include <rcl_action/action_server.h>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_waitables_interface.hpp>

#include <memory>
#include <string>

#include "rclcpp_action/server.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{
template<typename ACTION>
typename Server<ACTION>::SharedPtr
create_server(
  rclcpp::Node * node,
  const std::string & name,
  typename Server<ACTION>::GoalCallback handle_goal,
  typename Server<ACTION>::CancelCallback handle_cancel,
  typename Server<ACTION>::ExecuteCallback handle_execute,
  const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
{
  auto action_server = Server<ACTION>::make_shared(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    name,
    options,
    handle_goal,
    handle_cancel,
    handle_execute);

  // TODO(sloretz) shared pointer destructor should remove self from node waitables
  // TODO(sloretz) pass in callback group to this API
  node->get_node_waitables_interface()->add_waitable(action_server, nullptr);
  return action_server;
}
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CREATE_SERVER_HPP_
