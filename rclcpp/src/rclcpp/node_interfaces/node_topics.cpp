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

#include "rclcpp/node_interfaces/node_topics.hpp"

#include <string>

#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/exceptions.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeTopics;

NodeTopics::NodeTopics(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeTopics::~NodeTopics()
{}

rclcpp::PublisherBase::SharedPtr
NodeTopics::create_publisher(
  const std::string & topic_name,
  const rclcpp::PublisherFactory & publisher_factory,
  const rclcpp::QoS & qos)
{
  // Create the MessageT specific Publisher using the factory, but return it as PublisherBase.
  return publisher_factory.create_typed_publisher(node_base_, topic_name, qos);
}

void
NodeTopics::add_publisher(
  rclcpp::PublisherBase::SharedPtr publisher,
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group)
{
  // Assign to a group.
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      throw std::runtime_error("Cannot create publisher, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  for (auto & publisher_event : publisher->get_event_handlers()) {
    callback_group->add_waitable(publisher_event);
  }

  // Notify the executor that a new publisher was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to notify wait set on publisher creation: ") +
              rmw_get_error_string().str);
    }
  }
}

rclcpp::SubscriptionBase::SharedPtr
NodeTopics::create_subscription(
  const std::string & topic_name,
  const rclcpp::SubscriptionFactory & subscription_factory,
  const rclcpp::QoS & qos)
{
  // Create the MessageT specific Subscription using the factory, but return a SubscriptionBase.
  return subscription_factory.create_typed_subscription(node_base_, topic_name, qos);
}

void
NodeTopics::add_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription,
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group)
{
  // Assign to a group.
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create subscription, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  callback_group->add_subscription(subscription);
  for (auto & subscription_event : subscription->get_event_handlers()) {
    callback_group->add_waitable(subscription_event);
  }

  // Notify the executor that a new subscription was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
              std::string("Failed to notify wait set on subscription creation: ") +
              rmw_get_error_string().str
      );
    }
  }
}

rclcpp::node_interfaces::NodeBaseInterface *
NodeTopics::get_node_base_interface() const
{
  return node_base_;
}
