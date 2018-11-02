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
  rcl_publisher_options_t & publisher_options,
  bool use_intra_process)
{
  // Create the MessageT specific Publisher using the factory, but store it as PublisherBase.
  auto publisher = publisher_factory.create_typed_publisher(
    node_base_, topic_name, publisher_options);

  // Setup intra process publishing if requested.
  if (use_intra_process) {
    auto context = node_base_->get_context();
    // Get the intra process manager instance for this context.
    auto ipm = context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>();
    // Register the publisher with the intra process manager.
    uint64_t intra_process_publisher_id =
      publisher_factory.add_publisher_to_intra_process_manager(ipm.get(), publisher);
    // Create a function to be called when publisher to do the intra process publish.
    auto shared_publish_callback = publisher_factory.create_shared_publish_callback(ipm);
    publisher->setup_intra_process(
      intra_process_publisher_id,
      shared_publish_callback,
      publisher_options);
  }

  // Return the completed publisher.
  return publisher;
}

void
NodeTopics::add_publisher(
  rclcpp::PublisherBase::SharedPtr publisher)
{
  // The publisher is not added to a callback group or anthing like that for now.
  // It may be stored within the NodeTopics class or the NodeBase class in the future.
  (void)publisher;
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
  rcl_subscription_options_t & subscription_options,
  bool use_intra_process)
{
  auto subscription = subscription_factory.create_typed_subscription(
    node_base_, topic_name, subscription_options);

  // Setup intra process publishing if requested.
  if (use_intra_process) {
    auto context = node_base_->get_context();
    auto intra_process_manager =
      context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>();
    subscription_factory.setup_intra_process(
      intra_process_manager, subscription, subscription_options);
  }

  // Return the completed subscription.
  return subscription;
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
    callback_group->add_subscription(subscription);
  } else {
    node_base_->get_default_callback_group()->add_subscription(subscription);
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
