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

RCLCPP_LOCAL
inline
std::string
extend_name_with_sub_namespace(const std::string & name, const std::string & sub_namespace)
{
  std::string name_with_sub_namespace(name);
  if (sub_namespace != "" && name.front() != '/' && name.front() != '~') {
    name_with_sub_namespace = sub_namespace + "/" + name;
  }
  return name_with_sub_namespace;
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

  return rclcpp::create_publisher<MessageT, Alloc, PublisherT>(
    this->node_topics_.get(),
    extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
    qos_profile,
    this->get_node_options().use_intra_process_comms(),
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

  return rclcpp::create_subscription<MessageT, CallbackT, Alloc, CallbackMessageT, SubscriptionT>(
    this->node_topics_.get(),
    extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
    std::forward<CallbackT>(callback),
    qos_profile,
    group,
    ignore_local_publications,
    this->get_node_options().use_intra_process_comms(),
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

  auto cli = Client<ServiceT>::make_shared(
    node_base_.get(),
    node_graph_,
    extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
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
  return rclcpp::create_service<ServiceT, CallbackT>(
    node_base_,
    node_services_,
    extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
    std::forward<CallbackT>(callback),
    qos_profile,
    group);
}

template<typename ParameterT>
ParameterT
Node::declare_parameter(
  const std::string & name,
  const ParameterT & default_value,
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor)
{
  return this->declare_parameter(
    name,
    rclcpp::ParameterValue(default_value),
    parameter_descriptor
  ).get<ParameterT>();
}

template<typename ParameterT>
std::vector<ParameterT>
Node::declare_parameters(
  const std::string & namespace_,
  const std::map<std::string, ParameterT> & parameters)
{
  std::vector<ParameterT> result;
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");
  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(result),
    [this, &normalized_namespace](auto element) {
      return this->declare_parameter(normalized_namespace + element.first, element.second);
    }
  );
  return result;
}

template<typename ParameterT>
std::vector<ParameterT>
Node::declare_parameters(
  const std::string & namespace_,
  const std::map<
    std::string,
    std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>
  > & parameters)
{
  std::vector<ParameterT> result;
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");
  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(result),
    [this, &normalized_namespace](auto element) {
      return this->declare_parameter(
        normalized_namespace + element.first,
        element.second.first,
        element.second.second);
    }
  );
  return result;
}

template<typename ParameterT>
void
Node::set_parameter_if_not_set(
  const std::string & name,
  const ParameterT & value)
{
  if (!this->has_parameter(name)) {
    this->set_parameter(rclcpp::Parameter(name, value));
  }
}

// this is a partially-specialized version of set_parameter_if_not_set above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.
template<typename ParameterT>
void
Node::set_parameters_if_not_set(
  const std::string & name,
  const std::map<std::string, ParameterT> & values)
{
  std::vector<rclcpp::Parameter> params;

  for (const auto & val : values) {
    std::string parameter_name = name + "." + val.first;
    if (!this->has_parameter(parameter_name)) {
      params.push_back(rclcpp::Parameter(parameter_name, val.second));
    }
  }

  this->set_parameters(params);
}


template<typename ParameterT>
bool
Node::get_parameter(const std::string & name, ParameterT & parameter) const
{
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  rclcpp::Parameter parameter_variant;

  bool result = get_parameter(sub_name, parameter_variant);
  if (result) {
    parameter = parameter_variant.get_value<ParameterT>();
  }

  return result;
}

template<typename ParameterT>
bool
Node::get_parameter_or(
  const std::string & name,
  ParameterT & parameter,
  const ParameterT & alternative_value) const
{
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  bool got_parameter = get_parameter(sub_name, parameter);
  if (!got_parameter) {
    parameter = alternative_value;
  }
  return got_parameter;
}

// this is a partially-specialized version of get_parameter above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.
template<typename ParameterT>
bool
Node::get_parameters(
  const std::string & prefix,
  std::map<std::string, ParameterT> & values) const
{
  std::map<std::string, rclcpp::Parameter> params;
  bool result = node_parameters_->get_parameters_by_prefix(prefix, params);
  if (result) {
    for (const auto & param : params) {
      values[param.first] = param.second.get_value<ParameterT>();
    }
  }

  return result;
}

template<typename ParameterT>
void
Node::get_parameter_or_set(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value)
{
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  bool got_parameter = get_parameter(sub_name, value);
  if (!got_parameter) {
    this->set_parameters({
        rclcpp::Parameter(sub_name, alternative_value),
      });
    value = alternative_value;
  }
}

template<typename CallbackT>
void
Node::register_param_change_callback(CallbackT && callback)
{
  this->node_parameters_->register_param_change_callback(std::forward<CallbackT>(callback));
}

}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_
