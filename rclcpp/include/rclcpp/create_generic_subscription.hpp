// Copyright 2020, Apex.AI Inc.
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

#ifndef RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_
#define RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/typesupport_helpers.hpp"

namespace rclcpp
{

/// Create and return a GenericSubscription.
/**
 * The returned pointer will never be empty, but this function can throw various exceptions, for
 * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
 *
 * \param topics_interface NodeTopicsInterface pointer used in parts of the setup.
 * \param topic_name Topic name
 * \param topic_type Topic type
 * \param qos %QoS settings
 * \param callback Callback for new messages of serialized form
 * \param options %Publisher options.
 * Not all publisher options are currently respected, the only relevant options for this
 * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
 */
template<typename AllocatorT = std::allocator<void>>
std::shared_ptr<GenericSubscription> create_generic_subscription(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const std::string & topic_name,
  const std::string & topic_type,
  const rclcpp::QoS & qos,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  )
)
{
  auto ts_lib = rclcpp::get_typesupport_library(
    topic_type, "rosidl_typesupport_cpp");

  auto subscription = std::make_shared<GenericSubscription>(
    topics_interface->get_node_base_interface(),
    std::move(ts_lib),
    topic_name,
    topic_type,
    qos,
    callback,
    options);

  topics_interface->add_subscription(subscription, options.callback_group);

  return subscription;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_
