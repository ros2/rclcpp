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

#ifndef RCLCPP__CREATE_SUBSCRIPTION_HPP_
#define RCLCPP__CREATE_SUBSCRIPTION_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

/// Create and return a subscription of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface, or be a
 * NodeTopicsInterface pointer itself.
 */
template<
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename CallbackMessageT =
  typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT
  >,
  typename NodeT>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeT && node,
  const std::string & topic_name,
  const rosidl_message_type_support_t & type_support,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
  MessageMemoryStrategyT::create_default()
)
{
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(std::forward<NodeT>(node));

  auto factory = rclcpp::create_subscription_factory(
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat,
    type_support
  );

  auto sub = node_topics->create_subscription(topic_name, factory, qos);
  node_topics->add_subscription(sub, options.callback_group);

  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

/// Create and return a subscription of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface, or be a
 * NodeTopicsInterface pointer itself.
 */
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename CallbackMessageT =
  typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT
  >,
  typename NodeT>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeT && node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  ),
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
    MessageMemoryStrategyT::create_default()
  )
)
{
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(std::forward<NodeT>(node));

  auto factory = rclcpp::create_subscription_factory<MessageT>(
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat
  );

  auto sub = node_topics->create_subscription(topic_name, factory, qos);
  node_topics->add_subscription(sub, options.callback_group);

  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_
