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

#include "rclcpp/node_interfaces/node_services.hpp"

#include <string>

using rclcpp::node_interfaces::NodeServices;

NodeServices::NodeServices(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeServices::~NodeServices()
{}

void
NodeServices::add_service(
  rclcpp::ServiceBase::SharedPtr service_base_ptr,
  rclcpp::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(service_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_service(service_base_ptr);
  }

  // Notify the executor that a new service was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to notify wait set on service creation: ") +
              rmw_get_error_string().str
      );
    }
  }
}

void
NodeServices::add_client(
  rclcpp::ClientBase::SharedPtr client_base_ptr,
  rclcpp::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
    group->add_client(client_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_client(client_base_ptr);
  }

  // Notify the executor that a new client was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to notify wait set on client creation: ") +
              rmw_get_error_string().str
      );
    }
  }
}

std::string
NodeServices::resolve_service_name(const std::string & name, bool only_expand) const
{
  return node_base_->resolve_topic_or_service_name(name, true, only_expand);
}
