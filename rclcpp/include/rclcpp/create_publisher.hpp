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

#ifndef RCLCPP__CREATE_PUBLISHER_HPP_
#define RCLCPP__CREATE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/detail/qos_parameters.hpp"

#include "rmw/qos_profiles.h"

namespace rclcpp
{

namespace detail
{
// This is needed because it's currently possible to call `create_publisher()`
// only with a node topics interface.
// In that case, passing qos overridding options doesn't make sense.
//
// List of places in rclcpp passing directly a topic interface:
//  creating of "/parameters_event" topic in node parameters interface (this one is tricky).
//
// TODO(ivanpauno): Write overload in which you can pass
// a topic interface and a parameter interface directly.
// TODO2(ivanpauno): If we want the qos of `/parameters_event` topic to be reconfigurable,
// we need to figure out something.
template<typename NodeT>
std::enable_if_t<rclcpp::node_interfaces::has_node_parameters_interface<NodeT>::value, rclcpp::QoS>
get_actual_qos(
  const rclcpp::QosOverridingOptions & options, NodeT & node,
  std::string topic_name, rclcpp::QoS actual_qos)
{
  using rclcpp::node_interfaces::get_node_parameters_interface;
  if (options.policy_kinds.size()) {
    // TODO(ivanpauno)
    // Get expanded and remapped topic name before creating the node.
    // Need to refactor things in `rcl`.
    detail::declare_publisher_qos_parameters(
      options,
      *get_node_parameters_interface(node),
      topic_name,  // this should be the expanded and remapped topic name
      actual_qos);
  }
  return actual_qos;
}

template<typename NodeT>
std::enable_if_t<!rclcpp::node_interfaces::has_node_parameters_interface<NodeT>::value, rclcpp::QoS>
get_actual_qos(
  const rclcpp::QosOverridingOptions & options, NodeT, std::string, rclcpp::QoS actual_qos)
{
  if (options.policy_kinds.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "qos override options ignored because no parameter interface was provided");
  }
  return actual_qos;
}
}  // namespace detail

/// Create and return a publisher of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface.
 */
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
  typename NodeT>
std::shared_ptr<PublisherT>
create_publisher(
  NodeT & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  )
)
{
  // Extract the NodeTopicsInterface from the NodeT.
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(node);

  using rclcpp::node_interfaces::get_node_parameters_interface;
  rclcpp::QoS actual_qos = detail::get_actual_qos(
    options.qos_overriding_options, node, topic_name, qos);

  // Create the publisher.
  auto pub = node_topics->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(options),
    actual_qos
  );

  // Add the publisher to the node topics interface.
  node_topics->add_publisher(pub, options.callback_group);

  return std::dynamic_pointer_cast<PublisherT>(pub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_
