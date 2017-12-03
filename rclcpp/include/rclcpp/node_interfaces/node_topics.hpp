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
  explicit NodeTopics(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeTopics();

  RCLCPP_PUBLIC
  virtual
  rclcpp::PublisherBase::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rclcpp::PublisherFactory & publisher_factory,
    rcl_publisher_options_t & publisher_options,
    bool use_intra_process);

  RCLCPP_PUBLIC
  virtual
  void
  add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher);

  RCLCPP_PUBLIC
  virtual
  rclcpp::subscription::SubscriptionBase::SharedPtr
  create_subscription(
    const std::string & topic_name,
    const rclcpp::SubscriptionFactory & subscription_factory,
    rcl_subscription_options_t & subscription_options,
    bool use_intra_process);

  RCLCPP_PUBLIC
  virtual
  void
  add_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription,
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group);

private:
  RCLCPP_DISABLE_COPY(NodeTopics)

  NodeBaseInterface * node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_
