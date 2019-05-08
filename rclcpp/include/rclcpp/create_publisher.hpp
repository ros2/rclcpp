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

template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>>
// cppcheck-suppress syntaxError // bug in cppcheck 1.82 for [[deprecated]] on templated function
[[deprecated("use alternative rclcpp::create_publisher() signatures")]]
std::shared_ptr<PublisherT>
create_publisher(
  rclcpp::node_interfaces::NodeTopicsInterface * node_topics,
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  const PublisherEventCallbacks & event_callbacks,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool use_intra_process_comms,
  std::shared_ptr<AllocatorT> allocator)
{
  auto publisher_options = rcl_publisher_get_default_options();
  publisher_options.qos = qos_profile;

  auto pub = node_topics->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(event_callbacks, allocator),
    publisher_options,
    use_intra_process_comms);

  node_topics->add_publisher(pub, group);

  return std::dynamic_pointer_cast<PublisherT>(pub);
}

/// Create and return a publisher of the given MessageT type.
/**
 * The NodeT type only needs to have a method called get_node_topics_interface()
 * which returns a shared_ptr to a NodeTopicsInterface.
 */
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>,
  typename NodeT>
std::shared_ptr<PublisherT>
create_publisher(
  NodeT & node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
))
{
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(node);

  std::shared_ptr<AllocatorT> allocator = options.allocator;
  if (!allocator) {
    allocator = std::make_shared<AllocatorT>();
  }

  bool use_intra_process;
  switch (options.use_intra_process_comm) {
    case IntraProcessSetting::Enable:
      use_intra_process = true;
      break;
    case IntraProcessSetting::Disable:
      use_intra_process = false;
      break;
    case IntraProcessSetting::NodeDefault:
      use_intra_process = node_topics->get_node_base_interface()->get_use_intra_process_default();
      break;
    default:
      throw std::runtime_error("Unrecognized IntraProcessSetting value");
      break;
  }

  // TODO(wjwwood): convert all of the interfaces to use QoS and PublisherOptionsBase
  auto pub = node_topics->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(
      options.event_callbacks,
      allocator
    ),
    options.template to_rcl_publisher_options<MessageT>(qos),
    use_intra_process
  );
  node_topics->add_publisher(pub, options.callback_group);
  return std::dynamic_pointer_cast<PublisherT>(pub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_
