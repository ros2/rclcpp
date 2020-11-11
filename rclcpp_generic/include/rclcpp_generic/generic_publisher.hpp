// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__GENERIC_PUBLISHER_HPP_
#define ROSBAG2_TRANSPORT__GENERIC_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rosbag2_transport
{

class GenericPublisher : public rclcpp::PublisherBase
{
public:
  GenericPublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support,
    const std::string & topic_name,
    const rclcpp::QoS & qos);

  virtual ~GenericPublisher() = default;

  void publish(std::shared_ptr<rmw_serialized_message_t> message);
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__GENERIC_PUBLISHER_HPP_
