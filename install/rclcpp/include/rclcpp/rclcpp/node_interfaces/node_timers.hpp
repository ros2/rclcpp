// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeTimers part of the Node API.
class NodeTimers : public NodeTimersInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimers)

  RCLCPP_PUBLIC
  explicit NodeTimers(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeTimers();

  /// Add a timer to the node.
  RCLCPP_PUBLIC
  void
  add_timer(
    rclcpp::TimerBase::SharedPtr timer,
    rclcpp::CallbackGroup::SharedPtr callback_group) override;

private:
  RCLCPP_DISABLE_COPY(NodeTimers)

  rclcpp::node_interfaces::NodeBaseInterface * node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_
