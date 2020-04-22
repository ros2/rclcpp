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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/type_support_decl.hpp"

#include "lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rclcpp_lifecycle
{

template<typename MessageT, typename AllocatorT>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
LifecycleNode::create_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
{
  using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;
  return rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
    *this,
    topic_name,
    qos,
    options);
}

// TODO(karsten1987): Create LifecycleSubscriber
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT>
LifecycleNode::create_subscription(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat)
{
  return rclcpp::create_subscription<MessageT>(
    *this,
    topic_name,
    qos,
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat);
}

template<typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr
LifecycleNode::create_wall_timer(
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT callback,
  rclcpp::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::WallTimer<CallbackT>::make_shared(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::move(callback), this->node_base_->get_context());
  node_timers_->add_timer(timer, group);
  return timer;
}

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr
LifecycleNode::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::Client;
  using rclcpp::ClientBase;

  auto cli = Client<ServiceT>::make_shared(
    node_base_.get(),
    node_graph_,
    service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  node_services_->add_client(cli_base_ptr, group);
  return cli;
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
LifecycleNode::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return rclcpp::create_service<ServiceT, CallbackT>(
    node_base_, node_services_,
    service_name, std::forward<CallbackT>(callback), qos_profile, group);
}

template<typename ParameterT>
auto
LifecycleNode::declare_parameter(
  const std::string & name,
  const ParameterT & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor)
{
  return this->declare_parameter(
    name,
    rclcpp::ParameterValue(default_value),
    parameter_descriptor
  ).get<ParameterT>();
}

template<typename ParameterT>
std::vector<ParameterT>
LifecycleNode::declare_parameters(
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
LifecycleNode::declare_parameters(
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
      return static_cast<ParameterT>(
        this->declare_parameter(
          normalized_namespace + element.first,
          element.second.first,
          element.second.second)
      );
    }
  );
  return result;
}

template<typename ParameterT>
bool
LifecycleNode::get_parameter(const std::string & name, ParameterT & parameter) const
{
  rclcpp::Parameter param(name, parameter);
  bool result = get_parameter(name, param);
  parameter = param.get_value<ParameterT>();

  return result;
}

// this is a partially-specialized version of get_parameter above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.
template<typename MapValueT>
bool
LifecycleNode::get_parameters(
  const std::string & prefix,
  std::map<std::string, MapValueT> & values) const
{
  std::map<std::string, rclcpp::Parameter> params;
  bool result = node_parameters_->get_parameters_by_prefix(prefix, params);
  if (result) {
    for (const auto & param : params) {
      values[param.first] = param.second.get_value<MapValueT>();
    }
  }

  return result;
}

template<typename ParameterT>
bool
LifecycleNode::get_parameter_or(
  const std::string & name,
  ParameterT & value,
  const ParameterT & alternative_value) const
{
  bool got_parameter = get_parameter(name, value);
  if (!got_parameter) {
    value = alternative_value;
  }
  return got_parameter;
}

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
