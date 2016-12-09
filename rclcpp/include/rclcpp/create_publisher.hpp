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

#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp
{

template<typename MessageT, typename AllocatorT, typename PublisherT>
std::shared_ptr<PublisherT>
create_publisher(
  rclcpp::node_interfaces::NodeTopicsInterface * node_topics,
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  bool use_intra_process_comms,
  std::shared_ptr<AllocatorT> allocator)
{
  auto publisher_options = rcl_publisher_get_default_options();
  publisher_options.qos = qos_profile;

  auto pub = node_topics->create_publisher(
    topic_name,
    rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(allocator),
    publisher_options,
    use_intra_process_comms);
  node_topics->add_publisher(pub);
  return std::dynamic_pointer_cast<PublisherT>(pub);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_
