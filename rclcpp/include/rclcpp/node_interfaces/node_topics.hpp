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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_

#include <string>

#include "rcl/publisher.h"
#include "rcl/subscription.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeTopics part of the Node API.
class NodeTopics : public NodeTopicsInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTopicsInterface)

  RCLCPP_PUBLIC
  NodeTopics(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeTimersInterface * node_timers);

  RCLCPP_PUBLIC
  ~NodeTopics() override;

  RCLCPP_PUBLIC
  rclcpp::PublisherBase::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rclcpp::PublisherFactory & publisher_factory,
    const rclcpp::QoS & qos) override;

  RCLCPP_PUBLIC
  void
  add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher,
    rclcpp::CallbackGroup::SharedPtr callback_group) override;

  RCLCPP_PUBLIC
  rclcpp::SubscriptionBase::SharedPtr
  create_subscription(
    const std::string & topic_name,
    const rclcpp::SubscriptionFactory & subscription_factory,
    const rclcpp::QoS & qos) override;

  RCLCPP_PUBLIC
  void
  add_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription,
    rclcpp::CallbackGroup::SharedPtr callback_group) override;

  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface *
  get_node_base_interface() const override;

  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface *
  get_node_timers_interface() const override;

  RCLCPP_PUBLIC
  std::string
  resolve_topic_name(const std::string & name, bool only_expand = false) const override;

private:
  RCLCPP_DISABLE_COPY(NodeTopics)

  rclcpp::node_interfaces::NodeBaseInterface * node_base_;
  rclcpp::node_interfaces::NodeTimersInterface * node_timers_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_
