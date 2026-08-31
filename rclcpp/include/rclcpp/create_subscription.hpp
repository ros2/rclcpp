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

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

namespace detail
{
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT,
  typename NodeParametersT,
  typename NodeTopicsT
>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeParametersT & node_parameters,
  NodeTopicsT & node_topics,
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
  auto node_topics_interface = get_node_topics_interface(node_topics);

  auto factory = rclcpp::create_subscription_factory<MessageT>(
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat,
    options.subscription_statistics_monitor
  );

  const rclcpp::QoS & actual_qos = options.qos_overriding_options.get_policy_kinds().size() ?
    rclcpp::detail::declare_qos_parameters(
    options.qos_overriding_options, node_parameters,
    node_topics_interface->resolve_topic_name(topic_name),
    qos, rclcpp::detail::SubscriptionQosParametersTraits{}) :
    qos;

  auto sub = node_topics_interface->create_subscription(topic_name, factory, actual_qos);
  node_topics_interface->add_subscription(sub, options.callback_group);

  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}
}  // namespace detail

/// Create and return a subscription of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface, or be a
 * NodeTopicsInterface pointer itself.
 *
 * In case `options.qos_overriding_options` is enabling qos parameter overrides,
 * NodeT must also have a method called get_node_parameters_interface()
 * which returns a shared_ptr to a NodeParametersInterface.
 *
 * \tparam MessageT
 * \tparam CallbackT
 * \tparam AllocatorT
 * \tparam SubscriptionT
 * \tparam MessageMemoryStrategyT
 * \tparam NodeT
 * \param node
 * \param topic_name
 * \param qos
 * \param callback
 * \param options
 * \param msg_mem_strat
 * \return the created subscription
 * \throws std::invalid_argument if the QoS is incompatible with intra-process.
 */
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType,
  typename NodeT>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  NodeT & node,
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
  return rclcpp::detail::create_subscription<
    MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
    node, node, topic_name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
}

/// Create and return a subscription of the given MessageT type.
/**
 * See \ref create_subscription().
 */
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
typename std::shared_ptr<SubscriptionT>
create_subscription(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
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
  return rclcpp::detail::create_subscription<
    MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
    node_parameters, node_topics, topic_name, qos,
    std::forward<CallbackT>(callback), options, msg_mem_strat);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_
