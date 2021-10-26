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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/empty.hpp"

class TestCftSubscription : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_cft_subscription", "/ns");

    auto options = rclcpp::SubscriptionOptions();
    options.content_filter_options.filter_expression = "int32_value = %0";
    options.content_filter_options.expression_parameters = {"10"};

    auto callback = [](std::shared_ptr<const test_msgs::msg::BasicTypes>) {};
    sub = node->create_subscription<test_msgs::msg::BasicTypes>(
      "topic", 10, callback, options);
  }

  void TearDown()
  {
    sub.reset();
    node.reset();
  }

protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr sub;
};

TEST_F(TestCftSubscription, is_cft_enabled) {
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_enabled, false);
    EXPECT_FALSE(sub->is_cft_enabled());
  }

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_enabled, true);
    EXPECT_TRUE(sub->is_cft_enabled());
  }
}

TEST_F(TestCftSubscription, get_content_filter_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_get_content_filter, RCL_RET_ERROR);

  std::string filter_expression;
  std::vector<std::string> expression_parameters;
  EXPECT_THROW(
    sub->get_content_filter(filter_expression, expression_parameters),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestCftSubscription, set_content_filter_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_set_content_filter, RCL_RET_ERROR);

  std::string filter_expression = "int32_value = %0";
  std::string expression_parameter = "100";
  EXPECT_THROW(
    sub->set_content_filter(filter_expression, {expression_parameter}),
    rclcpp::exceptions::RCLError);
}
