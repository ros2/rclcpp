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

#include "rclcpp_generic/generic_publisher.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rclcpp_generic/typesupport_helpers.hpp"

namespace
{
rcl_publisher_options_t rosbag2_get_publisher_options(const rclcpp::QoS & qos)
{
  auto options = rcl_publisher_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
}  // unnamed namespace

namespace rclcpp_generic
{

std::shared_ptr<GenericPublisher> GenericPublisher::create(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  auto ts_lib = rclcpp_generic::get_typesupport_library(
    topic_type, "rosidl_typesupport_cpp");
  // Cannot use make_shared because constructor is private
  std::shared_ptr<GenericPublisher> pub(new GenericPublisher(
      topics_interface->get_node_base_interface(), std::move(ts_lib), topic_name, topic_type, qos));
  topics_interface->add_publisher(pub, std::move(group));
  return pub;
}

GenericPublisher::GenericPublisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
  const std::string & topic_name,
  const std::string & topic_type,
  const rclcpp::QoS & qos)
: rclcpp::PublisherBase(node_base, topic_name, *rclcpp_generic::get_typesupport_handle(
      topic_type, "rosidl_typesupport_cpp", ts_lib), rosbag2_get_publisher_options(qos)), ts_lib_(
    ts_lib)
{}

void GenericPublisher::publish(std::shared_ptr<rmw_serialized_message_t> message)
{
  auto return_code = rcl_publish_serialized_message(
    get_publisher_handle().get(), message.get(), NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
  }
}

}  // namespace rclcpp_generic
