// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/node.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

using namespace std::chrono_literals;

class TestCreateSubscription : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestCreateSubscription, create) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    rclcpp::create_subscription<test_msgs::msg::Empty>(node, "topic_name", qos, callback, options);

  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
}

TEST_F(TestCreateSubscription, create_with_overriding_options) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    rclcpp::create_subscription<test_msgs::msg::Empty>(node, "topic_name", qos, callback, options);

  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
}

TEST_F(TestCreateSubscription, create_separated_node_topics_and_parameters) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};

  auto node_parameters = node->get_node_parameters_interface();
  auto node_topics = node->get_node_topics_interface();
  auto subscription = rclcpp::create_subscription<test_msgs::msg::Empty>(
    node_parameters, node_topics, "topic_name", qos, callback, options);

  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
}

TEST_F(TestCreateSubscription, create_with_statistics) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  options.topic_stats_options.publish_topic = "topic_statistics";
  options.topic_stats_options.publish_period = 5min;

  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    rclcpp::create_subscription<test_msgs::msg::Empty>(node, "topic_name", qos, callback, options);

  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
}

TEST_F(TestCreateSubscription, create_with_intra_process_com) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto options = rclcpp::SubscriptionOptions();
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  ASSERT_NO_THROW(
  {
    subscription = rclcpp::create_subscription<test_msgs::msg::Empty>(
      node, "topic_name", rclcpp::SystemDefaultsQoS(), callback, options);
  });
  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
}
