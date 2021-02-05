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

#include "rclcpp/detail/resolve_enable_topic_statistics.hpp"

#include "rclcpp/node_interfaces/get_node_timers_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

namespace detail
{
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT,
  typename SubscriptionT,
  typename MessageMemoryStrategyT,
  typename NodeParametersT,
  typename NodeTopicsT>
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

  std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<CallbackMessageT>>
  subscription_topic_stats = nullptr;

  if (rclcpp::detail::resolve_enable_topic_statistics(
      options,
      *node_topics_interface->get_node_base_interface()))
  {
    if (options.topic_stats_options.publish_period <= std::chrono::milliseconds(0)) {
      throw std::invalid_argument(
              "topic_stats_options.publish_period must be greater than 0, specified value of " +
              std::to_string(options.topic_stats_options.publish_period.count()) +
              " ms");
    }

    std::shared_ptr<Publisher<statistics_msgs::msg::MetricsMessage>>
    publisher = rclcpp::detail::create_publisher<statistics_msgs::msg::MetricsMessage>(
      node_parameters,
      node_topics_interface,
      options.topic_stats_options.publish_topic,
      qos);

    subscription_topic_stats = std::make_shared<
      rclcpp::topic_statistics::SubscriptionTopicStatistics<CallbackMessageT>
      >(node_topics_interface->get_node_base_interface()->get_name(), publisher);

    std::weak_ptr<
      rclcpp::topic_statistics::SubscriptionTopicStatistics<CallbackMessageT>
    > weak_subscription_topic_stats(subscription_topic_stats);
    auto sub_call_back = [weak_subscription_topic_stats]() {
        auto subscription_topic_stats = weak_subscription_topic_stats.lock();
        if (subscription_topic_stats) {
          subscription_topic_stats->publish_message_and_reset_measurements();
        }
      };

    auto node_timer_interface = node_topics_interface->get_node_timers_interface();

    auto timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        options.topic_stats_options.publish_period),
      sub_call_back,
      options.callback_group,
      node_topics_interface->get_node_base_interface(),
      node_timer_interface
    );

    subscription_topic_stats->set_publisher_timer(timer);
  }

  auto factory = rclcpp::create_subscription_factory<MessageT>(
    std::forward<CallbackT>(callback),
    options,
    msg_mem_strat,
    subscription_topic_stats
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
 * \tparam CallbackMessageT
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
 * \throws std::invalid_argument if topic statistics is enabled and the publish period is
 * less than or equal to zero.
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
    MessageT, CallbackT, AllocatorT, CallbackMessageT, SubscriptionT, MessageMemoryStrategyT>(
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
  typename CallbackMessageT =
  typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT
  >>
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
    MessageT, CallbackT, AllocatorT, CallbackMessageT, SubscriptionT, MessageMemoryStrategyT>(
    node_parameters, node_topics, topic_name, qos,
    std::forward<CallbackT>(callback), options, msg_mem_strat);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_
