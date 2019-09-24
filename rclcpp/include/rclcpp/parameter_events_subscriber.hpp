// Copyright 2019 Intel Corporation
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

#ifndef RCLCPP__PARAMETER_EVENTS_SUBSCRIBER_HPP_
#define RCLCPP__PARAMETER_EVENTS_SUBSCRIBER_HPP_

#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace rclcpp
{

class ParameterEventsSubscriber
{
public:
  RCLCPP_PUBLIC
  ParameterEventsSubscriber(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const rclcpp::QoS & qos = rclcpp::ParameterEventsQoS());

  template<typename NodeT>
  ParameterEventsSubscriber(
    NodeT node,
    const rclcpp::QoS & qos =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)))
  : ParameterEventsSubscriber(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface(),
      qos)
  {}

  /// Set a custom callback for parameter events.
  /**
   * If no namespace is provided, a subscription will be created for the current namespace.
   * Repeated calls to this function will overwrite the callback.
   * If more than one namespace already has a subscription to its parameter events topic, then the
   * provided callback will be applied to all of them.
   *
   * \param[in] callback Function callback to be evaluated on event.
   * \param[in] node_namespace Name of namespace for which a subscription will be created.
   */
  RCLCPP_PUBLIC
  void
  set_event_callback(
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
    const std::string & node_namespace = "");

  /// Add a custom callback for a specified parameter.
  /**
   * If a node_name is not provided, defaults to the current node.
   *
   * \param[in] parameter_name Name of parameter.
   * \param[in] callback Function callback to be evaluated upon parameter event.
   * \param[in] node_name Name of node which hosts the parameter.
   */
  RCLCPP_PUBLIC
  void
  register_parameter_callback(
    const std::string & parameter_name,
    std::function<void(const rclcpp::Parameter &)> callback,
    const std::string & node_name = "");

  RCLCPP_PUBLIC
  static bool
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    rclcpp::Parameter & parameter,
    const std::string parameter_name,
    const std::string node_name = "");

  RCLCPP_PUBLIC
  static rclcpp::Parameter
  get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event,
    const std::string parameter_name,
    const std::string node_name = "");

protected:
  /// Add a subscription (if unique) to a namespace parameter events topic.
  void
  add_namespace_event_subscriber(const std::string & node_namespace);

  /// Callback for parameter events subscriptions.
  void
  event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  // Utility functions for string and path name operations.
  std::string resolve_path(const std::string & path);
  std::pair<std::string, std::string> split_path(const std::string & str);
  std::string join_path(std::string path, std::string name);

  // Node Interfaces used for logging and creating subscribers.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  rclcpp::QoS qos_;

  // *INDENT-OFF* Uncrustify doesn't handle indented public/private labels
  // Hash function for string pair required in std::unordered_map
  class StringPairHash
  {
  public:
    template<typename T>
    inline void hash_combine(std::size_t & seed, const T & v) const
    {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    inline size_t operator()(const std::pair<std::string, std::string> & s) const
    {
      size_t seed = 0;
      hash_combine(seed, s.first);
      hash_combine(seed, s.second);
      return seed;
    }
  };
  // *INDENT-ON*

  // Map container for registered parameters.
  std::unordered_map<
    std::pair<std::string, std::string>,
    std::function<void(const rclcpp::Parameter &)>,
    StringPairHash
  > parameter_callbacks_;

  // Vector of unique namespaces added.
  std::vector<std::string> node_namespaces_;

  // Vector of event subscriptions for each namespace.
  std::vector<rclcpp::Subscription
    <rcl_interfaces::msg::ParameterEvent>::SharedPtr> event_subscriptions_;

  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> user_callback_;

  std::recursive_mutex mutex_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_EVENTS_SUBSCRIBER_HPP_
