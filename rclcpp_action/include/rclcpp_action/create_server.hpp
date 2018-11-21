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

#include <rclcpp/node.hpp>

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
  typename Server<ACTION>::CancelCallback handle_cancel)
{
  return Server<ACTION>::make_shared(
    node->get_node_base_interface(),
    name,
    handle_goal,
    handle_cancel);
}
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CREATE_SERVER_HPP_
