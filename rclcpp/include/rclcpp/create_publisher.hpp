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
/// Create and return a publisher of the given MessageT type.
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
  typename NodeParametersT,
  typename NodeTopicsT>
std::shared_ptr<PublisherT>
create_publisher(
  NodeParametersT & node_parameters,
  NodeTopicsT & node_topics,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  )
)
{
  auto node_topics_interface = rclcpp::node_interfaces::get_node_topics_interface(node_topics);
  const rclcpp::QoS & actual_qos = options.qos_overriding_options.get_policy_kinds().size() ?
    rclcpp::detail::declare_qos_parameters(
    options.qos_overriding_options, node_parameters,
    node_topics_interface->resolve_topic_name(topic_name),
    qos, rclcpp::detail::PublisherQosParametersTraits{}) :
    qos;

  // Create the publisher.
  auto pub = node_topics_interface->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(options),
    actual_qos
  );

  // Add the publisher to the node topics interface.
  node_topics_interface->add_publisher(pub, options.callback_group);

  return std::dynamic_pointer_cast<PublisherT>(pub);
}
}  // namespace detail


/// Create and return a publisher of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface.
 *
 * In case `options.qos_overriding_options` is enabling qos parameter overrides,
 * NodeT must also have a method called get_node_parameters_interface()
 * which returns a shared_ptr to a NodeParametersInterface.
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
  return detail::create_publisher<MessageT, AllocatorT, PublisherT>(
    node, node, topic_name, qos, options);
}

/// Create and return a publisher of the given MessageT type.
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
std::shared_ptr<PublisherT>
create_publisher(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  )
)
{
  return detail::create_publisher<MessageT, AllocatorT, PublisherT>(
    node_parameters, node_topics, topic_name, qos, options);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_
