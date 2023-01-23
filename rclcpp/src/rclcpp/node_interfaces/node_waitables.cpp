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

#include "rclcpp/node_interfaces/node_waitables.hpp"

#include <string>

using rclcpp::node_interfaces::NodeWaitables;

NodeWaitables::NodeWaitables(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeWaitables::~NodeWaitables()
{}

void
NodeWaitables::add_waitable(
  rclcpp::Waitable::SharedPtr waitable_ptr,
  rclcpp::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacobperron): use custom exception
      throw std::runtime_error("Cannot create waitable, group not in node.");
    }
  } else {
    group = node_base_->get_default_callback_group();
  }

  group->add_waitable(waitable_ptr);

  // Notify the executor that a new waitable was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on waitable creation: ") + ex.what());
  }
}

void
NodeWaitables::remove_waitable(
  rclcpp::Waitable::SharedPtr waitable_ptr,
  rclcpp::CallbackGroup::SharedPtr group) noexcept
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      return;
    }
    group->remove_waitable(waitable_ptr);
  } else {
    node_base_->get_default_callback_group()->remove_waitable(waitable_ptr);
  }
}
