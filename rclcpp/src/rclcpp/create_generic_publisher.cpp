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

#include "rclcpp/create_generic_publisher.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/typesupport_helpers.hpp"

namespace rclcpp
{

std::shared_ptr<GenericPublisher> create_generic_publisher(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  auto ts_lib = rclcpp::get_typesupport_library(
    topic_type, "rosidl_typesupport_cpp");
  auto pub = std::make_shared<GenericPublisher>(
    topics_interface->get_node_base_interface(), std::move(ts_lib), topic_name, topic_type, qos);
  topics_interface->add_publisher(pub, std::move(group));
  return pub;
}

}  // namespace rclcpp
