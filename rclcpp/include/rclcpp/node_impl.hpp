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
#include <chrono>
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

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/create_client.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/detail/resolve_enable_topic_statistics.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__NODE_HPP_
#include "node.hpp"
#endif

namespace rclcpp
{

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

template<typename MessageT, typename AllocatorT, typename PublisherT>
std::shared_ptr<PublisherT>
Node::create_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const PublisherOptionsWithAllocator<AllocatorT> & options)
{
  return rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
    *this,
    extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
    qos,
    options);
}

template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT>
Node::create_subscription(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const SubscriptionOptionsWithAllocator<AllocatorT> & options,
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat)
{
  return rclcpp::create_subscription<MessageT>(
    *this,
    extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
    qos,
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat);
}

template<typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr
Node::create_wall_timer(
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT callback,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return rclcpp::create_wall_timer(
    period,
    std::move(callback),
    group,
    this->node_base_.get(),
    this->node_timers_.get());
}

template<typename ServiceT>
typename Client<ServiceT>::SharedPtr
Node::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return rclcpp::create_client<ServiceT>(
    node_base_,
    node_graph_,
    node_services_,
    extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
    qos_profile,
    group);
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
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
auto
Node::declare_parameter(
  const std::string & name,
  const ParameterT & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  try {
    return this->declare_parameter(
      name,
      rclcpp::ParameterValue(default_value),
      parameter_descriptor,
      ignore_override
    ).get<ParameterT>();
  } catch (const ParameterTypeException & ex) {
    throw exceptions::InvalidParameterTypeException(name, ex.what());
  }
}

template<typename ParameterT>
std::vector<ParameterT>
Node::declare_parameters(
  const std::string & namespace_,
  const std::map<std::string, ParameterT> & parameters,
  bool ignore_overrides)
{
  std::vector<ParameterT> result;
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");
  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(result),
    [this, &normalized_namespace, ignore_overrides](auto element) {
      return this->declare_parameter(
        normalized_namespace + element.first,
        element.second,
        rcl_interfaces::msg::ParameterDescriptor(),
        ignore_overrides);
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
  > & parameters,
  bool ignore_overrides)
{
  std::vector<ParameterT> result;
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");
  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(result),
    [this, &normalized_namespace, ignore_overrides](auto element) {
      return static_cast<ParameterT>(
        this->declare_parameter(
          normalized_namespace + element.first,
          element.second.first,
          element.second.second,
          ignore_overrides)
      );
    }
  );
  return result;
}

template<typename ParameterT>
bool
Node::get_parameter(const std::string & name, ParameterT & parameter) const
{
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  rclcpp::Parameter parameter_variant;

  bool result = get_parameter(sub_name, parameter_variant);
  if (result) {
    parameter = static_cast<ParameterT>(parameter_variant.get_value<ParameterT>());
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
      values[param.first] = static_cast<ParameterT>(param.second.get_value<ParameterT>());
    }
  }

  return result;
}

}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_
