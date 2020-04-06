// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "test_msgs/msg/empty.hpp"

class TestQosEvent : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_qos_event", "/ns");
    is_fastrtps =
      std::string(rmw_get_implementation_identifier()).find("rmw_fastrtps") != std::string::npos;
  }

  void TearDown()
  {
    node.reset();
  }

  static constexpr char topic_name[] = "test_topic";
  rclcpp::Node::SharedPtr node;
  bool is_fastrtps;
};

constexpr char TestQosEvent::topic_name[];

/*
   Testing construction of a publishers with QoS event callback functions.
 */
TEST_F(TestQosEvent, test_publisher_constructor)
{
  rclcpp::PublisherOptions options;

  // options arg with no callbacks
  auto publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with one of the callbacks
  options.event_callbacks.deadline_callback =
    [node = node.get()](rclcpp::QOSDeadlineOfferedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Offered deadline missed - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with two of the callbacks
  options.event_callbacks.liveliness_callback =
    [node = node.get()](rclcpp::QOSLivelinessLostInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Liveliness lost - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  publisher = node->create_publisher<test_msgs::msg::Empty>(
    topic_name, 10, options);

  // options arg with three of the callbacks
  options.event_callbacks.incompatible_qos_callback =
    [node = node.get()](rclcpp::QOSOfferedIncompatibleQoSInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Offered incompatible qos - total %d (delta %d), last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };
  try {
    publisher = node->create_publisher<test_msgs::msg::Empty>(
      topic_name, 10, options);
  } catch (const rclcpp::UnsupportedEventTypeException & /*exc*/) {
    EXPECT_TRUE(is_fastrtps);
  }
}

/*
   Testing construction of a subscriptions with QoS event callback functions.
 */
TEST_F(TestQosEvent, test_subscription_constructor)
{
  rclcpp::SubscriptionOptions options;

  auto message_callback = [node = node.get()](const test_msgs::msg::Empty::SharedPtr /*msg*/) {
      RCLCPP_INFO(node->get_logger(), "Message received");
    };

  // options arg with no callbacks
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with one of the callbacks
  options.event_callbacks.deadline_callback =
    [node = node.get()](rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Requested deadline missed - total %d (delta %d)",
        event.total_count, event.total_count_change);
    };
  subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with two of the callbacks
  options.event_callbacks.liveliness_callback =
    [node = node.get()](rclcpp::QOSLivelinessChangedInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Liveliness changed - alive %d (delta %d), not alive %d (delta %d)",
        event.alive_count, event.alive_count_change,
        event.not_alive_count, event.not_alive_count_change);
    };
  subscription = node->create_subscription<test_msgs::msg::Empty>(
    topic_name, 10, message_callback, options);

  // options arg with three of the callbacks
  options.event_callbacks.incompatible_qos_callback =
    [node = node.get()](rclcpp::QOSRequestedIncompatibleQoSInfo & event) {
      RCLCPP_INFO(
        node->get_logger(),
        "Requested incompatible qos - total %d (delta %d), last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
    };
  try {
    subscription = node->create_subscription<test_msgs::msg::Empty>(
      topic_name, 10, message_callback, options);
  } catch (const rclcpp::UnsupportedEventTypeException & /*exc*/) {
    EXPECT_TRUE(is_fastrtps);
  }
}
