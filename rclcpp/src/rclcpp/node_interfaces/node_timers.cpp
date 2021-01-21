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

#include "rclcpp/node_interfaces/node_timers.hpp"

#include <string>

#include "tracetools/tracetools.h"

using rclcpp::node_interfaces::NodeTimers;

NodeTimers::NodeTimers(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeTimers::~NodeTimers()
{}

void
NodeTimers::add_timer(
  rclcpp::TimerBase::SharedPtr timer,
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
    callback_group->add_timer(timer);
  } else {
    node_base_->get_default_callback_group()->add_timer(timer);
  }
  if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
    throw std::runtime_error(
            std::string("Failed to notify wait set on timer creation: ") +
            rmw_get_error_string().str);
  }
  TRACEPOINT(
    rclcpp_timer_link_node,
    static_cast<const void *>(timer->get_timer_handle().get()),
    static_cast<const void *>(node_base_->get_rcl_node_handle()));
}
