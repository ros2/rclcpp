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
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__NODE_HPP_
#include "node.hpp"
#endif

namespace rclcpp
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

  std::string extended_topic_name(topic_name);
  if (extended_namespace_ != "" && topic_name.front() != '/' && topic_name.front() != '~') {
    extended_topic_name = extended_namespace_ + "/" + topic_name;
  }

  return rclcpp::create_publisher<MessageT, Alloc, PublisherT>(
    this->node_topics_.get(),
    extended_topic_name,
    qos_profile,
    use_intra_process_comms_,
    allocator);
}

template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename SubscriptionT>
std::shared_ptr<SubscriptionT>
Node::create_subscription(
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
  using CallbackMessageT = typename rclcpp::subscription_traits::has_message_type<CallbackT>::type;

  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }

  if (!msg_mem_strat) {
    using rclcpp::message_memory_strategy::MessageMemoryStrategy;
    msg_mem_strat = MessageMemoryStrategy<CallbackMessageT, Alloc>::create_default();
  }

  std::string extended_topic_name(topic_name);
  if (extended_namespace_ != "" && topic_name.front() != '/' && topic_name.front() != '~') {
    extended_topic_name = extended_namespace_ + "/" + topic_name;
  }

  return rclcpp::create_subscription<MessageT, CallbackT, Alloc, CallbackMessageT, SubscriptionT>(
    this->node_topics_.get(),
    extended_topic_name,
    std::forward<CallbackT>(callback),
    qos_profile,
    group,
    ignore_local_publications,
    use_intra_process_comms_,
    msg_mem_strat,
    allocator);
}

template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename SubscriptionT>
std::shared_ptr<SubscriptionT>
Node::create_subscription(
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
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;

  return this->create_subscription<MessageT>(
    topic_name,
    std::forward<CallbackT>(callback),
    qos,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

template<typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr
Node::create_wall_timer(
  std::chrono::duration<int64_t, DurationT> period,
  CallbackT callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::WallTimer<CallbackT>::make_shared(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::move(callback),
    this->node_base_->get_context());
  node_timers_->add_timer(timer, group);
  return timer;
}

template<typename ServiceT>
typename Client<ServiceT>::SharedPtr
Node::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::Client;
  using rclcpp::ClientBase;

  std::string extended_service_name(service_name);
  if (extended_namespace_ != "" && service_name.front() != '/' && service_name.front() != '~') {
    extended_service_name = extended_namespace_ + "/" + service_name;
  }

  auto cli = Client<ServiceT>::make_shared(
    node_base_.get(),
    node_graph_,
    extended_service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  node_services_->add_client(cli_base_ptr, group);
  return cli;
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  std::string extended_service_name(service_name);
  if (extended_namespace_ != "" && service_name.front() != '/' && service_name.front() != '~') {
    extended_service_name = extended_namespace_ + "/" + service_name;
  }

  return rclcpp::create_service<ServiceT, CallbackT>(
    node_base_, node_services_,
    extended_service_name, std::forward<CallbackT>(callback), qos_profile, group);
}

template<typename CallbackT>
void
Node::register_param_change_callback(CallbackT && callback)
{
  this->node_parameters_->register_param_change_callback(std::forward<CallbackT>(callback));
}

template<typename ParameterT>
void
Node::set_parameter_if_not_set(
  const std::string & name,
  const ParameterT & value)
{
  std::string extended_name(name);
  if (extended_namespace_ != "" && name.front() != '/' && name.front() != '~') {
    extended_name = extended_namespace_ + "/" + name;
  }

  rclcpp::Parameter parameter;
  if (!this->get_parameter(extended_name, parameter)) {
    this->set_parameters({
        rclcpp::Parameter(extended_name, value),
      });
  }
}

// this is a partially-specialized version of set_parameter_if_not_set above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.
template<typename MapValueT>
void
Node::set_parameters_if_not_set(
  const std::string & name,
  const std::map<std::string, MapValueT> & values)
{
  std::vector<rclcpp::Parameter> params;

  for (const auto & val : values) {
    std::string param_name = name + "." + val.first;
    rclcpp::Parameter parameter;
    if (!this->get_parameter(param_name, parameter)) {
      params.push_back(rclcpp::Parameter(param_name, val.second));
    }
  }

  this->set_parameters(params);
}

template<typename ParameterT>
bool
Node::get_parameter(const std::string & name, ParameterT & value) const
{
  std::string extended_name(name);
  if (extended_namespace_ != "" && name.front() != '/' && name.front() != '~') {
    extended_name = extended_namespace_ + "/" + name;
  }

  rclcpp::Parameter parameter;

  bool result = get_parameter(extended_name, parameter);
  if (result) {
    value = parameter.get_value<ParameterT>();
  }

  return result;
}

// this is a partially-specialized version of get_parameter above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.
template<typename MapValueT>
bool
Node::get_parameters(
  const std::string & name,
  std::map<std::string, MapValueT> & values) const
{
  std::map<std::string, rclcpp::Parameter> params;
  bool result = node_parameters_->get_parameters_by_prefix(name, params);
  if (result) {
    for (const auto & param : params) {
      values[param.first] = param.second.get_value<MapValueT>();
    }
  }

  return result;
}

template<typename ParameterT>
bool
Node::get_parameter_or(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value) const
{
  std::string extended_name(name);
  if (extended_namespace_ != "" && name.front() != '/' && name.front() != '~') {
    extended_name = extended_namespace_ + "/" + name;
  }

  bool got_parameter = get_parameter(extended_name, value);
  if (!got_parameter) {
    value = alternative_value;
  }
  return got_parameter;
}

template<typename ParameterT>
void
Node::get_parameter_or_set(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value)
{
  std::string extended_name(name);
  if (extended_namespace_ != "" && name.front() != '/' && name.front() != '~') {
    extended_name = extended_namespace_ + "/" + name;
  }

  bool got_parameter = get_parameter(extended_name, value);
  if (!got_parameter) {
    this->set_parameters({
        rclcpp::Parameter(extended_name, alternative_value),
      });
    value = alternative_value;
  }
}

}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_
