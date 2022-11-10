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

#include <stdexcept>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/qos.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeTopics;

NodeTopics::NodeTopics(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  rclcpp::node_interfaces::NodeTimersInterface * node_timers)
: node_base_(node_base), node_timers_(node_timers)
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
  rclcpp::CallbackGroup::SharedPtr callback_group)
{
  // Assign to a group.
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      throw std::runtime_error("Cannot create publisher, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  for (auto & key_event_pair : publisher->get_event_handlers()) {
    auto publisher_event = key_event_pair.second;
    callback_group->add_waitable(publisher_event);
  }

  // Notify the executor that a new publisher was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on publisher creation: ") + ex.what());
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
  rclcpp::CallbackGroup::SharedPtr callback_group)
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

  for (auto & key_event_pair : subscription->get_event_handlers()) {
    auto subscription_event = key_event_pair.second;
    callback_group->add_waitable(subscription_event);
  }

  auto intra_process_waitable = subscription->get_intra_process_waitable();
  if (nullptr != intra_process_waitable) {
    // Add to the callback group to be notified about intra-process msgs.
    callback_group->add_waitable(intra_process_waitable);
  }

  // Notify the executor that a new subscription was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on subscription creation: ") + ex.what());
  }
}

rclcpp::node_interfaces::NodeBaseInterface *
NodeTopics::get_node_base_interface() const
{
  return node_base_;
}

rclcpp::node_interfaces::NodeTimersInterface *
NodeTopics::get_node_timers_interface() const
{
  return node_timers_;
}

std::string
NodeTopics::resolve_topic_name(const std::string & name, bool only_expand) const
{
  return node_base_->resolve_topic_or_service_name(name, false, only_expand);
}
