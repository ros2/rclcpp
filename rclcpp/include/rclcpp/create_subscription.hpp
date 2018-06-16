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

#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  rclcpp::node_interfaces::NodeTopicsInterface * node_topics,
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
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
    std::forward<CallbackT>(callback), msg_mem_strat, allocator);

  auto sub = node_topics->create_subscription(
    topic_name,
    factory,
    subscription_options,
    use_intra_process_comms);
  node_topics->add_subscription(sub, group);
  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_
