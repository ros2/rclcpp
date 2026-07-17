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
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_statistics_monitor.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"
#include "rmw/types.h"

class TestSubscriptionOptions : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("test_subscription_options", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

class MockSubscriptionStatisticsMonitor : public rclcpp::SubscriptionStatisticsMonitor
{
public:
  void before_message_dispatch(const rmw_message_info_t &) override
  {
    before_message_dispatch_count++;
  }
  void after_message_dispatch(const rmw_message_info_t &) override
  {
    after_message_dispatch_count++;
  }

  size_t before_message_dispatch_count{0};
  size_t after_message_dispatch_count{0};
};

TEST_F(TestSubscriptionOptions, subscription_statistics_monitor_default_and_set) {
  auto options = rclcpp::SubscriptionOptions();
  EXPECT_EQ(nullptr, options.subscription_statistics_monitor);

  auto monitor = std::make_shared<MockSubscriptionStatisticsMonitor>();
  options.subscription_statistics_monitor = monitor;
  EXPECT_EQ(monitor, options.subscription_statistics_monitor);
}
