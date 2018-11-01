// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUB_NODE_IMPL_HPP_
#define RCLCPP__SUB_NODE_IMPL_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl/publisher.h"
#include "rcl/subscription.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__SUB_NODE_HPP_
#include "sub_node.hpp"
#endif

namespace rclcpp
{

template<typename MessageT, typename Alloc, typename PublisherT>
std::shared_ptr<PublisherT>
SubNode::create_publisher(
  const std::string & topic_name, size_t qos_history_depth,
  std::shared_ptr<Alloc> allocator)
{
  std::string new_topic_name = add_extended_ns_prefix(topic_name);

  return original_->create_publisher<MessageT, Alloc, PublisherT>(
    new_topic_name, qos_history_depth, allocator);
}

template<typename MessageT, typename Alloc, typename PublisherT>
std::shared_ptr<PublisherT>
SubNode::create_publisher(
  const std::string & topic_name, const rmw_qos_profile_t & qos_profile,
  std::shared_ptr<Alloc> allocator)
{
  std::string new_topic_name = add_extended_ns_prefix(topic_name);

  return original_->create_publisher<MessageT, Alloc, PublisherT>(
    new_topic_name, qos_profile, allocator);
}

template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename SubscriptionT>
std::shared_ptr<SubscriptionT>
SubNode::create_subscription(
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  std::string new_topic_name = add_extended_ns_prefix(topic_name);

  return original_->create_subscription<MessageT, CallbackT, Alloc, SubscriptionT>(
    new_topic_name,
    std::forward<CallbackT>(callback),
    qos_profile,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename SubscriptionT>
std::shared_ptr<SubscriptionT>
SubNode::create_subscription(
  const std::string & topic_name,
  CallbackT && callback,
  size_t qos_history_depth,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  std::string new_topic_name = add_extended_ns_prefix(topic_name);

  return original_->create_subscription<MessageT, CallbackT, Alloc, SubscriptionT>(
    new_topic_name,
    std::forward<CallbackT>(callback),
    qos_history_depth,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

template<typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr
SubNode::create_wall_timer(
  std::chrono::duration<int64_t, DurationT> period,
  CallbackT callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  original_->create_wall_timer<DurationT, CallbackT>(
    period, std::forward<CallbackT>(callback), group);
}

template<typename ServiceT>
typename Client<ServiceT>::SharedPtr
SubNode::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  std::string new_service_name = add_extended_ns_prefix(service_name);

  return original_->create_client<ServiceT>(new_service_name, qos_profile, group);
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
SubNode::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  std::string new_service_name = add_extended_ns_prefix(service_name);

  return original_->create_service<ServiceT, CallbackT>(
    new_service_name,
    std::forward<CallbackT>(callback),
    group);
}

template<typename CallbackT>
void
SubNode::register_param_change_callback(CallbackT && callback)
{
  original_->register_param_change_callback(std::forward<CallbackT>(callback));
}

template<typename ParameterT>
void
SubNode::set_parameter_if_not_set(
  const std::string & name,
  const ParameterT & value)
{
  std::string new_parameter_name = add_extended_ns_prefix(name);
  original_->set_parameter_if_not_set<ParameterT>(new_parameter_name, value);
}

template<typename ParameterT>
bool
SubNode::get_parameter(const std::string & name, ParameterT & value) const
{
  std::string new_parameter_name = add_extended_ns_prefix(name);

  return original_->get_parameter<ParameterT>(new_parameter_name, value);
}

template<typename ParameterT>
bool
SubNode::get_parameter_or(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value) const
{
  std::string new_parameter_name = add_extended_ns_prefix(name);

  return original_->get_parameter_or<ParameterT>(new_parameter_name, value, alternative_value);
}

template<typename ParameterT>
void
SubNode::get_parameter_or_set(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value)
{
  std::string new_parameter_name = add_extended_ns_prefix(name);

  return original_->get_parameter_or_set<ParameterT>(new_parameter_name, value, alternative_value);
}

}  // namespace rclcpp

#endif  // RCLCPP__SUB_NODE_IMPL_HPP_
