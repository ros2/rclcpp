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

#include <memory>

#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialized_message.hpp"
#include "test_msgs/msg/empty.hpp"

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

TEST_F(TestCreateSubscription, create_with_deprecated_callback) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto sub_node = node->create_sub_node("sub");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](test_msgs::msg::Empty::SharedPtr) {};
  auto serialized_callback = [](std::shared_ptr<rclcpp::SerializedMessage>) {};
  auto node_parameters = node->get_node_parameters_interface();
  auto node_topics = node->get_node_topics_interface();

#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  auto node_subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback);
  auto sub_node_subscription =
    sub_node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback);
  auto subscription =
    rclcpp::create_subscription<test_msgs::msg::Empty>(node, "topic_name", qos, callback, options);
  auto separated_subscription = rclcpp::create_subscription<test_msgs::msg::Empty>(
    node_parameters, node_topics, "topic_name", qos, callback, options);
  auto node_generic_subscription = node->create_generic_subscription(
    "generic_topic_name", "test_msgs/msg/Empty", qos, serialized_callback);
  auto generic_subscription = rclcpp::create_generic_subscription(
    node_topics, "generic_topic_name", "test_msgs/msg/Empty", qos, serialized_callback);
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

  ASSERT_NE(nullptr, node_subscription);
  EXPECT_STREQ("/ns/topic_name", node_subscription->get_topic_name());
  ASSERT_NE(nullptr, sub_node_subscription);
  EXPECT_STREQ("/ns/sub/topic_name", sub_node_subscription->get_topic_name());
  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
  ASSERT_NE(nullptr, separated_subscription);
  EXPECT_STREQ("/ns/topic_name", separated_subscription->get_topic_name());
  ASSERT_NE(nullptr, node_generic_subscription);
  EXPECT_STREQ("/ns/generic_topic_name", node_generic_subscription->get_topic_name());
  ASSERT_NE(nullptr, generic_subscription);
  EXPECT_STREQ("/ns/generic_topic_name", generic_subscription->get_topic_name());
}
