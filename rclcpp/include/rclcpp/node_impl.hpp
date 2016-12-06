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

#ifndef RCLCPP__NODE_IMPL_HPP_
#define RCLCPP__NODE_IMPL_HPP_

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
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__NODE_HPP_
#include "node.hpp"
#endif

namespace rclcpp
{
namespace node
{

template<typename MessageT, typename Alloc, typename PublisherT>
std::shared_ptr<PublisherT>
Node::create_publisher(
  const std::string & topic_name, size_t qos_history_depth,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_publisher<MessageT, Alloc, PublisherT>(topic_name, qos, allocator);
}

template<typename MessageT, typename Alloc, typename PublisherT>
std::shared_ptr<PublisherT>
Node::create_publisher(
  const std::string & topic_name, const rmw_qos_profile_t & qos_profile,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }

  auto publisher_options = rcl_publisher_get_default_options();
  publisher_options.qos = qos_profile;

  auto pub = node_topics_->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, Alloc, PublisherT>(allocator),
    publisher_options,
    use_intra_process_comms_);
  node_topics_->add_publisher(
    pub,
    // this is the callback group, not currently used or exposed to the user for publishers
    nullptr);
  return std::dynamic_pointer_cast<PublisherT>(pub);
}

template<typename MessageT, typename CallbackT, typename Alloc>
typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  typename std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }

  if (!msg_mem_strat) {
    msg_mem_strat =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::create_default();
  }

  auto subscription_options = rcl_subscription_get_default_options();
  subscription_options.qos = qos_profile;
  subscription_options.ignore_local_publications = ignore_local_publications;

  using SubscriptionT = rclcpp::subscription::Subscription<MessageT, Alloc>;
  auto factory = rclcpp::create_subscription_factory<MessageT, CallbackT, Alloc, SubscriptionT>(
    std::forward<CallbackT>(callback), msg_mem_strat, allocator);

  auto sub = node_topics_->create_subscription(
    topic_name,
    factory,
    subscription_options,
    use_intra_process_comms_);
  node_topics_->add_subscription(sub, group);
  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

template<typename MessageT, typename CallbackT, typename Alloc>
typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  size_t qos_history_depth,
  CallbackT && callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_subscription<MessageT, CallbackT, Alloc>(
    topic_name,
    std::forward<CallbackT>(callback),
    qos,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

template<typename CallbackType>
typename rclcpp::timer::WallTimer<CallbackType>::SharedPtr
Node::create_wall_timer(
  std::chrono::nanoseconds period,
  CallbackType callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::timer::WallTimer<CallbackType>::make_shared(
    period, std::move(callback));
  if (group) {
    if (!group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
    group->add_timer(timer);
  } else {
    node_base_->get_default_callback_group()->add_timer(timer);
  }
  number_of_timers_++;
  if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
    throw std::runtime_error(
            std::string(
              "Failed to notify waitset on timer creation: ") + rmw_get_error_string());
  }
  return timer;
}

template<typename ServiceT>
typename client::Client<ServiceT>::SharedPtr
Node::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::client::Client;
  using rclcpp::client::ClientBase;

  auto cli = Client<ServiceT>::make_shared(
    shared_from_this(),
    service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  node_services_->add_client(cli_base_ptr, group);
  return cli;
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::service::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rclcpp::service::AnyServiceCallback<ServiceT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos_profile;

  auto serv = service::Service<ServiceT>::make_shared(
    node_base_->get_shared_rcl_node_handle(),
    service_name, any_service_callback, service_options);
  auto serv_base_ptr = std::dynamic_pointer_cast<service::ServiceBase>(serv);
  node_services_->add_service(serv_base_ptr, group);
  return serv;
}

template<typename CallbackT>
void
Node::register_param_change_callback(CallbackT && callback)
{
  this->node_parameters_->register_param_change_callback(std::forward<CallbackT>(callback));
}

template<typename ParameterT>
bool
Node::get_parameter(const std::string & name, ParameterT & parameter) const
{
  rclcpp::parameter::ParameterVariant parameter_variant(name, parameter);
  bool result = get_parameter(name, parameter_variant);
  parameter = parameter_variant.get_value<ParameterT>();

  return result;
}

}  // namespace node
}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_
