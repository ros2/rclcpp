// Copyright 2021, Apex.AI Inc.
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

#include "rclcpp/generic/create_generic_subscription.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"
#include "rclcpp/generic/typesupport_helpers.hpp"

namespace rclcpp
{
namespace generic
{

std::shared_ptr<GenericSubscription> create_generic_subscription(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const std::string & topic_name,
  const std::string & topic_type,
  const rclcpp::QoS & qos,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
  rclcpp::CallbackGroup::SharedPtr group)
{
  auto ts_lib = rclcpp::generic::get_typesupport_library(
    topic_type, "rosidl_typesupport_cpp");

  // Cannot use make_shared because constructor is private
  std::shared_ptr<GenericSubscription> subscription(new GenericSubscription(
      topics_interface->get_node_base_interface(),
      std::move(ts_lib),
      topic_name,
      topic_type,
      qos,
      callback));

  topics_interface->add_subscription(subscription, std::move(group));

  return subscription;
}

}  // namespace generic
}  // namespace rclcpp
