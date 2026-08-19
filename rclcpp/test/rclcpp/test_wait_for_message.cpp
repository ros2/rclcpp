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
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace std::chrono_literals;

class TestWaitForMessage : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    // Unique names avoid cross-talk across tests / RMWs that share a domain.
    node_name_ = std::string("wait_for_message_") + info->name();
    topic_name_ = std::string("wait_for_message_topic_") + info->name();
    node_ = std::make_shared<rclcpp::Node>(node_name_);
  }

  void TearDown() override
  {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string node_name_;
  std::string topic_name_;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestWaitForMessage, wait_for_message) {
  using MsgT = test_msgs::msg::Strings;
  auto pub = node_->create_publisher<MsgT>(topic_name_, 10);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node_, topic_name_, 5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }
  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);
}

TEST_F(TestWaitForMessage, wait_for_message_indefinitely) {
  using MsgT = test_msgs::msg::Strings;
  MsgT out;
  auto received = false;
  // Create the subscription while the context is still valid. Some RMWs (e.g. zenoh)
  // throw if wait_for_message tries to create a subscription after shutdown.
  auto sub = node_->create_subscription<MsgT>(
    topic_name_, 10, [](const std::shared_ptr<const MsgT>) {});
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, sub, node_->get_node_options().context());
      EXPECT_FALSE(ret);
      received = ret;
    });

  // Let the waiter enter wait_set.wait before interrupting via shutdown.
  std::this_thread::sleep_for(100ms);
  rclcpp::shutdown();

  ASSERT_NO_THROW(wait.get());
  ASSERT_FALSE(received);
}

TEST_F(TestWaitForMessage, wait_for_message_twice_one_sub) {
  using MsgT = test_msgs::msg::Strings;
  auto pub = node_->create_publisher<MsgT>(topic_name_, 10);
  auto sub = node_->create_subscription<MsgT>(
    topic_name_, 1, [](const std::shared_ptr<const MsgT>) {});

  MsgT out1;
  MsgT out2;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out1, sub, node_->get_node_options().context(), 5s);
      EXPECT_TRUE(ret);
      ret = rclcpp::wait_for_message(out2, sub, node_->get_node_options().context(), 5s);
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
}

TEST_F(TestWaitForMessage, wait_for_last_message) {
  auto qos = rclcpp::QoS(1).reliable().transient_local();

  using MsgT = test_msgs::msg::Strings;
  auto pub = node_->create_publisher<MsgT>(topic_name_, qos);
  pub->publish(*get_messages_strings()[0]);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node_, topic_name_, 5s, qos);
      EXPECT_TRUE(ret);
      received = true;
    });

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);
}

TEST_F(TestWaitForMessage, wait_for_last_message_with_unbounded_uint8_values) {
  auto qos = rclcpp::QoS(1).reliable().transient_local();

  using MsgT = test_msgs::msg::UnboundedSequences;
  auto pub = node_->create_publisher<MsgT>(topic_name_, qos);
  MsgT input;
  input.uint8_values = {1, 2, 3};
  input.alignment_check = 42;
  pub->publish(input);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node_, topic_name_, 5s, qos);
      EXPECT_TRUE(ret);
      received = true;
    });

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, input);
}

TEST(TestWaitForMessageCustomContext, wait_for_message_custom_context) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string node_name = std::string("wait_for_message_custom_context_") + info->name();
  const std::string topic_name = std::string("wait_for_message_topic_") + info->name();

  auto node_opt = rclcpp::NodeOptions().context(context);
  auto node = std::make_shared<rclcpp::Node>(node_name, node_opt);

  using MsgT = test_msgs::msg::Strings;
  auto pub = node->create_publisher<MsgT>(topic_name, 10);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(out, node, topic_name, 5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }
  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);

  node.reset();
  context->shutdown("test complete");
}

TEST_F(TestWaitForMessage, wait_for_message_explicit_interfaces) {
  using MsgT = test_msgs::msg::Strings;
  auto pub = node_->create_publisher<MsgT>(topic_name_, 10);

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      auto ret = rclcpp::wait_for_message(
        out,
        node_->get_node_parameters_interface(),
        node_->get_node_topics_interface(),
        topic_name_,
        5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }
  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);
}
