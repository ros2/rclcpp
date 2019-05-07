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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcl/publisher.h"
#include "rcl/subscription.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeTopics part of the Node API.
class NodeTopicsInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTopicsInterface)

  RCLCPP_PUBLIC
  virtual
  ~NodeTopicsInterface() = default;

  RCLCPP_PUBLIC
  virtual
  rclcpp::PublisherBase::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rclcpp::PublisherFactory & publisher_factory,
    const rcl_publisher_options_t & publisher_options,
    bool use_intra_process) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher,
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group) = 0;

  RCLCPP_PUBLIC
  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_subscription(
    const std::string & topic_name,
    const rclcpp::SubscriptionFactory & subscription_factory,
    const rcl_subscription_options_t & subscription_options,
    bool use_intra_process) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  add_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription,
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group) = 0;

  RCLCPP_PUBLIC
  virtual
  rclcpp::node_interfaces::NodeBaseInterface *
  get_node_base_interface() const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_
