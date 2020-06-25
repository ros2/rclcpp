// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

static const rosidl_message_type_support_t empty_type_support =
  *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Empty>();

static const rcl_publisher_options_t publisher_options =
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>().template
  to_rcl_publisher_options<test_msgs::msg::Empty>(rclcpp::QoS(10));

static const rcl_subscription_options_t subscription_options =
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>().template
  to_rcl_subscription_options<test_msgs::msg::Empty>(rclcpp::QoS(10));


class TestPublisher : public rclcpp::PublisherBase
{
public:
  explicit TestPublisher(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(), "topic", empty_type_support, publisher_options) {}
};

class TestSubscription : public rclcpp::SubscriptionBase
{
public:
  explicit TestSubscription(rclcpp::Node * node)
  : rclcpp::SubscriptionBase(
      node->get_node_base_interface().get(), empty_type_support, "topic", subscription_options) {}
  std::shared_ptr<void> create_message() override {return nullptr;}

  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override {return nullptr;}

  void handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &) override {}
  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override {}
  void return_message(std::shared_ptr<void> &) override {}
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &) override {}
};


TEST(TestNodeService, add_publisher)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto * node_topics =
    dynamic_cast<rclcpp::node_interfaces::NodeTopics *>(node->get_node_topics_interface().get());
  ASSERT_NE(nullptr, node_topics);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto publisher = std::make_shared<TestPublisher>(node.get());
  EXPECT_THROW(
    node_topics->add_publisher(publisher, callback_group),
    std::runtime_error);

  rclcpp::shutdown();
}

TEST(TestNodeService, add_subscription)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");
  auto * node_topics =
    dynamic_cast<rclcpp::node_interfaces::NodeTopics *>(node->get_node_topics_interface().get());
  ASSERT_NE(nullptr, node_topics);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscription = std::make_shared<TestSubscription>(node.get());
  EXPECT_THROW(
    node_topics->add_subscription(subscription, callback_group),
    std::runtime_error);

  rclcpp::shutdown();
}
