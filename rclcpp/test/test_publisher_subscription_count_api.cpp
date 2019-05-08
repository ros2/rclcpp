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
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

using rcl_interfaces::msg::IntraProcessMessage;

/**
 * Parameterized test.
 * The first param are the NodeOptions used to create the nodes.
 * The second param are the expect intraprocess count results.
 */
struct TestParameters
{
  rclcpp::NodeOptions node_options[2];
  uint64_t intraprocess_count_results[2];
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestPublisherSubscriptionCount : public ::testing::TestWithParam<TestParameters>
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

std::chrono::milliseconds TestPublisherSubscriptionCount::offset = std::chrono::milliseconds(2000);

void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
{
  (void)msg;
}

TEST_P(TestPublisherSubscriptionCount, increasing_and_decreasing_counts)
{
  TestParameters parameters = GetParam();
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
    "my_node",
    "/ns",
    parameters.node_options[0]);
  auto publisher = node->create_publisher<IntraProcessMessage>("/topic", 10);

  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(publisher->get_intra_process_subscription_count(), 0u);
  {
    auto sub = node->create_subscription<IntraProcessMessage>("/topic", 10, &OnMessage);
    rclcpp::sleep_for(offset);
    EXPECT_EQ(publisher->get_subscription_count(), 1u);
    EXPECT_EQ(
      publisher->get_intra_process_subscription_count(),
      parameters.intraprocess_count_results[0]);
    {
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        parameters.node_options[1]);
      auto another_sub =
        another_node->create_subscription<IntraProcessMessage>("/topic", 10, &OnMessage);

      rclcpp::sleep_for(offset);
      EXPECT_EQ(publisher->get_subscription_count(), 2u);
      EXPECT_EQ(
        publisher->get_intra_process_subscription_count(),
        parameters.intraprocess_count_results[1]);
    }
    rclcpp::sleep_for(offset);
    EXPECT_EQ(publisher->get_subscription_count(), 1u);
    EXPECT_EQ(
      publisher->get_intra_process_subscription_count(),
      parameters.intraprocess_count_results[0]);
  }
  /**
    * Counts should be zero here, as all are subscriptions are out of scope.
    * Subscriptions count checking is always preceeded with an sleep, as random failures had been
    * detected without it. */
  rclcpp::sleep_for(offset);
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(publisher->get_intra_process_subscription_count(), 0u);
}

auto get_new_context()
{
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  return context;
}

TestParameters parameters[] = {
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two subscriptions in the same topic, both using intraprocess comm.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().use_intra_process_comms(true)
    },
    {1u, 2u},
    "two_subscriptions_intraprocess_comm"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two subscriptions, one using intra-process comm and the other not using it.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().use_intra_process_comms(false)
    },
    {1u, 1u},
    "two_subscriptions_one_intraprocess_one_not"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two contexts, both using intra-process.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(true)
    },
    {1u, 1u},
    "two_subscriptions_in_two_contexts_with_intraprocess_comm"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two contexts, both of them not using intra-process comm.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(false),
      rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(false)
    },
    {0u, 0u},
    "two_subscriptions_in_two_contexts_without_intraprocess_comm"
  }
};

INSTANTIATE_TEST_CASE_P(
  TestWithDifferentNodeOptions, TestPublisherSubscriptionCount,
  ::testing::ValuesIn(parameters),
  ::testing::PrintToStringParamName());
