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

#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

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

  // Create the publisher.
  auto pub = node_topics->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(options),
    qos
  );

  // Add the publisher to the node topics interface.
  node_topics->add_publisher(pub, options.callback_group);

  return std::dynamic_pointer_cast<PublisherT>(pub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_
