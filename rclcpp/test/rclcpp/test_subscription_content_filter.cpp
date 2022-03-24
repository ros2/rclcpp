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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/basic_types.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestContentFilterSubscription, RMW_IMPLEMENTATION) : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_content_filter_node", "/ns");

    auto options = rclcpp::SubscriptionOptions();
    auto callback = [](std::shared_ptr<const test_msgs::msg::BasicTypes>) {};
    sub = node->create_subscription<test_msgs::msg::BasicTypes>(
      "content_filter_topic", qos.reliable().transient_local(), callback, options);
  }

  void TearDown()
  {
    sub.reset();
    node.reset();
  }

protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr sub;
};

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), get_content_filter_error) {
  std::string filter_expression;
  std::vector<std::string> expression_parameters;
  EXPECT_THROW(
    sub->get_content_filter(filter_expression, expression_parameters),
    rclcpp::exceptions::RCLError);
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), set_content_filter_error) {
  std::string filter_expression = "int32_value = %0";
  std::string expression_parameter = "100";
  EXPECT_THROW(
    sub->set_content_filter(filter_expression, {expression_parameter}),
    rclcpp::exceptions::RCLError);
}
