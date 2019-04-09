// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

class TestPubSubOptionAPI : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestPubSubOptionAPI, check_for_ambiguous) {
  rclcpp::PublisherOptions<> pub_options;
  rclcpp::SubscriptionOptions<> sub_options;

  auto topic_only_pub = node->create_publisher<test_msgs::msg::Empty>(
    "topic_only");
  auto topic_depth_pub = node->create_publisher<test_msgs::msg::Empty>(
    "topic_depth",
    10);
  auto all_options_pub = node->create_publisher<test_msgs::msg::Empty>(
    "topic_options",
    10,
    pub_options);

  auto topic_only_sub = node->create_subscription<test_msgs::msg::Empty>(
    "topic_only",
    [](std::shared_ptr<test_msgs::msg::Empty> test_msg) {(void) test_msg;});
  auto topic_depth_sub = node->create_subscription<test_msgs::msg::Empty>(
    "topic_depth",
    [](std::shared_ptr<test_msgs::msg::Empty> test_msg) {(void) test_msg;},
    10);
  auto all_options_sub = node->create_subscription<test_msgs::msg::Empty>(
    "topic_options",
    [](std::shared_ptr<test_msgs::msg::Empty> test_msg) {(void) test_msg;},
    10,
    sub_options);
}
