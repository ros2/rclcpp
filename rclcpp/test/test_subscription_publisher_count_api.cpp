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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

using rcl_interfaces::msg::IntraProcessMessage;

void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
{
  (void)msg;
}

class TestSubscriptionPublisherCount : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>(
      "my_node",
      "/ns");
    subscription = node->create_subscription<IntraProcessMessage>("/topic", &OnMessage);
  }

  void TearDown()
  {
    node.reset();
  }

  void test_common(rclcpp::NodeOptions options, const uint64_t intraprocess_count_results[2]);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::SubscriptionBase> subscription;
  static std::chrono::milliseconds offset;
};

std::chrono::milliseconds TestSubscriptionPublisherCount::offset = std::chrono::milliseconds(2000);

void TestSubscriptionPublisherCount::test_common(rclcpp::NodeOptions options)
{
  EXPECT_EQ(subscription->get_publisher_count(), 0u);
  {
    auto pub = node->create_publisher<IntraProcessMessage>("/topic");
    rclcpp::sleep_for(offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
    {
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        options);
      auto another_pub =
        another_node->create_publisher<IntraProcessMessage>("/topic");

      rclcpp::sleep_for(offset);
      EXPECT_EQ(subscription->get_publisher_count(), 2u);
    }
    rclcpp::sleep_for(offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
  }
  rclcpp::sleep_for(offset);
  EXPECT_EQ(subscription->get_publisher_count(), 0u);
}

/*
   Testing subscription publisher count api.
   One context.
 */
TEST_F(TestSubscriptionPublisherCount, test_one_context) {
  test_common(rclcpp::NodeOptions());
}

/*
   Testing subscription publisher count api.
   Two contexts.
 */
TEST_F(TestSubscriptionPublisherCount, test_two_contexts) {
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  test_common(
    rclcpp::NodeOptions().context(context),
    results);
}
