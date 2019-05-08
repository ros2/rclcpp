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

#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

using rcl_interfaces::msg::IntraProcessMessage;

struct TestParameters
{
  rclcpp::NodeOptions node_options;
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestSubscriptionPublisherCount : public ::testing::TestWithParam<TestParameters>
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

protected:
  void SetUp() {}
  void TearDown() {}
  static std::chrono::milliseconds offset;
};

std::chrono::milliseconds TestSubscriptionPublisherCount::offset = std::chrono::milliseconds(2000);

void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
{
  (void)msg;
}

TEST_P(TestSubscriptionPublisherCount, increasing_and_decreasing_counts)
{
  rclcpp::NodeOptions node_options = GetParam().node_options;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
    "my_node",
    "/ns",
    node_options);
  auto subscription = node->create_subscription<IntraProcessMessage>("/topic", 10, &OnMessage);

  EXPECT_EQ(subscription->get_publisher_count(), 0u);
  {
    auto pub = node->create_publisher<IntraProcessMessage>("/topic", 10);
    rclcpp::sleep_for(offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
    {
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        node_options);
      auto another_pub =
        another_node->create_publisher<IntraProcessMessage>("/topic", 10);

      rclcpp::sleep_for(offset);
      EXPECT_EQ(subscription->get_publisher_count(), 2u);
    }
    rclcpp::sleep_for(offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
  }
  rclcpp::sleep_for(offset);
  EXPECT_EQ(subscription->get_publisher_count(), 0u);
}

auto get_new_context()
{
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  return context;
}

TestParameters parameters[] = {
  /*
     Testing subscription publisher count api.
     One context.
   */
  {
    rclcpp::NodeOptions(),
    "one_context_test"
  },
  /*
     Testing subscription publisher count api.
     Two contexts.
   */
  {
    rclcpp::NodeOptions().context(get_new_context()),
    "two_contexts_test"
  }
};

INSTANTIATE_TEST_CASE_P(
  TestWithDifferentNodeOptions,
  TestSubscriptionPublisherCount,
  testing::ValuesIn(parameters),
  testing::PrintToStringParamName());
