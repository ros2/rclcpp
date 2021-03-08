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

#include "rclcpp/generic_publisher.hpp"

#include <memory>
#include <string>

#include "rclcpp/typesupport_helpers.hpp"

namespace rclcpp
{

namespace
{
rcl_publisher_options_t get_publisher_options(const rclcpp::QoS & qos)
{
  auto options = rcl_publisher_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
}  // unnamed namespace

GenericPublisher::GenericPublisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
  const std::string & topic_name,
  const std::string & topic_type,
  const rclcpp::QoS & qos)
: rclcpp::PublisherBase(
    node_base,
    topic_name,
    *rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib),
    get_publisher_options(qos)),
  ts_lib_(ts_lib)
{}


void GenericPublisher::publish(const rclcpp::SerializedMessage & message)
{
  auto return_code = rcl_publish_serialized_message(
    get_publisher_handle().get(), &message.get_rcl_serialized_message(), NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
  }
}

}  // namespace rclcpp
