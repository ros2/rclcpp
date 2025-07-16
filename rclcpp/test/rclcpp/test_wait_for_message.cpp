// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "test_msgs/msg/strings.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace std::chrono_literals;

TEST(TestUtilities, wait_for_message) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("wait_for_message_node");

  using MsgT = test_msgs::msg::Strings;
  auto pub = node->create_publisher<MsgT>("wait_for_message_topic", 10);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node, "wait_for_message_topic", 5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);

  rclcpp::shutdown();
}

TEST(TestUtilities, wait_for_message_indefinitely) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("wait_for_message_node2");

  using MsgT = test_msgs::msg::Strings;
  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node, "wait_for_message_topic" /*, -1 */);
      EXPECT_TRUE(ret);
      received = true;
    });

  rclcpp::shutdown();

  ASSERT_FALSE(received);
}

TEST(TestUtilities, wait_for_message_twice_one_sub) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("wait_for_message_node3");

  using MsgT = test_msgs::msg::Strings;
  auto pub = node->create_publisher<MsgT>("wait_for_message_topic", 10);
  auto sub = node->create_subscription<MsgT>(
    "wait_for_message_topic", 1, [](const std::shared_ptr<const MsgT>) {});

  MsgT out1;
  MsgT out2;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out1, sub, node->get_node_options().context(), 5s);
      EXPECT_TRUE(ret);
      ret = rclcpp::wait_for_message(out2, sub, node->get_node_options().context(), 5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out1, *get_messages_strings()[0]);
  EXPECT_EQ(out2, *get_messages_strings()[0]);

  rclcpp::shutdown();
}

TEST(TestUtilities, wait_for_last_message) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("wait_for_last_message_node");
  auto qos = rclcpp::QoS(1).reliable().transient_local();

  using MsgT = test_msgs::msg::Strings;
  auto pub = node->create_publisher<MsgT>("wait_for_last_message_topic", qos);
  pub->publish(*get_messages_strings()[0]);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node, "wait_for_last_message_topic", 5s, qos);
      EXPECT_TRUE(ret);
      received = true;
    });

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);

  rclcpp::shutdown();
}
