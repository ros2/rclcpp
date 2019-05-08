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

template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>>
// cppcheck-suppress syntaxError // bug in cppcheck 1.82 for [[deprecated]] on templated function
[[deprecated("use alternative rclcpp::create_subscription() signatures")]]
typename std::shared_ptr<SubscriptionT>
create_subscription(
  rclcpp::node_interfaces::NodeTopicsInterface * node_topics,
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  const SubscriptionEventCallbacks & event_callbacks,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  bool use_intra_process_comms,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT, AllocatorT>::SharedPtr
  msg_mem_strat,
  typename std::shared_ptr<AllocatorT> allocator)
{
  auto subscription_options = rcl_subscription_get_default_options();
  subscription_options.qos = qos_profile;
  subscription_options.ignore_local_publications = ignore_local_publications;

  auto factory = rclcpp::create_subscription_factory
    <MessageT, CallbackT, AllocatorT, CallbackMessageT, SubscriptionT>(
    std::forward<CallbackT>(callback),
    event_callbacks,
    msg_mem_strat,
    allocator);

  auto sub = node_topics->create_subscription(
    topic_name,
    factory,
    subscription_options,
    use_intra_process_comms);
  node_topics->add_subscription(sub, group);
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
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT, AllocatorT>::SharedPtr
  msg_mem_strat = nullptr)
{
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(std::forward<NodeT>(node));

  if (!msg_mem_strat) {
    using rclcpp::message_memory_strategy::MessageMemoryStrategy;
    msg_mem_strat = MessageMemoryStrategy<CallbackMessageT, AllocatorT>::create_default();
  }

  std::shared_ptr<AllocatorT> allocator = options.allocator;
  if (!allocator) {
    allocator = std::make_shared<AllocatorT>();
  }
  auto factory = rclcpp::create_subscription_factory
    <MessageT, CallbackT, AllocatorT, CallbackMessageT, SubscriptionT>(
    std::forward<CallbackT>(callback), options.event_callbacks, msg_mem_strat, allocator);

  bool use_intra_process;
  switch (options.use_intra_process_comm) {
    case IntraProcessSetting::Enable:
      use_intra_process = true;
      break;
    case IntraProcessSetting::Disable:
      use_intra_process = false;
      break;
    case IntraProcessSetting::NodeDefault:
      use_intra_process = node_topics->get_node_base_interface()->get_use_intra_process_default();
      break;
    default:
      throw std::runtime_error("Unrecognized IntraProcessSetting value");
      break;
  }

  // TODO(wjwwood): convert all of the interfaces to use QoS and SubscriptionOptionsBase
  auto sub = node_topics->create_subscription(
    topic_name,
    factory,
    options.template to_rcl_subscription_options<MessageT>(qos),
    use_intra_process);
  node_topics->add_subscription(sub, options.callback_group);
  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_
