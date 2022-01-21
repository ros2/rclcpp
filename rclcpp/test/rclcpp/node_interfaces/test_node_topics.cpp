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
#include <string>
#include <type_traits>
#include <vector>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

namespace
{

const rosidl_message_type_support_t EmptyTypeSupport()
{
  return *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Empty>();
}

const rcl_publisher_options_t PublisherOptions()
{
  return rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>().template
         to_rcl_publisher_options<test_msgs::msg::Empty>(rclcpp::QoS(10));
}

const rcl_subscription_options_t SubscriptionOptions()
{
  return rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>().template
         to_rcl_subscription_options<test_msgs::msg::Empty>(rclcpp::QoS(10));
}

}  // namespace

class TestPublisher : public rclcpp::PublisherBase
{
public:
  explicit TestPublisher(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(), "topic", EmptyTypeSupport(), PublisherOptions()) {}
};

class TestSubscription : public rclcpp::SubscriptionBase
{
public:
  explicit TestSubscription(rclcpp::Node * node)
  : rclcpp::SubscriptionBase(
      node->get_node_base_interface().get(), EmptyTypeSupport(), "topic", SubscriptionOptions()) {}
  std::shared_ptr<void> create_message() override {return nullptr;}

  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override {return nullptr;}

  void handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &) override {}
  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override {}
  void handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &) override {}
  void return_message(std::shared_ptr<void> &) override {}
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &) override {}
};

class TestNodeTopics : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options{};
    options.arguments(std::vector<std::string>{"-r", "foo:=bar"});
    node = std::make_shared<rclcpp::Node>("node", "ns", options);

    // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
    // the proper type is being tested and covered.
    node_topics =
      dynamic_cast<rclcpp::node_interfaces::NodeTopics *>(node->get_node_topics_interface().get());
    ASSERT_NE(nullptr, node_topics);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::node_interfaces::NodeTopics * node_topics;
};

TEST_F(TestNodeTopics, add_publisher)
{
  auto publisher = std::make_shared<TestPublisher>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NO_THROW(node_topics->add_publisher(publisher, callback_group));

  // Check that adding publisher from node to a callback group in different_node throws exception.
  std::shared_ptr<rclcpp::Node> different_node = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group_in_different_node =
    different_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_THROW(
    node_topics->add_publisher(publisher, callback_group_in_different_node),
    std::runtime_error);
}

TEST_F(TestNodeTopics, add_publisher_rcl_trigger_guard_condition_error)
{
  auto publisher = std::make_shared<TestPublisher>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_topics->add_publisher(publisher, callback_group),
    std::runtime_error("failed to notify wait set on publisher creation: error not set"));
}

TEST_F(TestNodeTopics, add_subscription)
{
  auto subscription = std::make_shared<TestSubscription>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NO_THROW(node_topics->add_subscription(subscription, callback_group));

  // Check that adding subscription from node to callback group in different_node throws exception.
  std::shared_ptr<rclcpp::Node> different_node = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group_in_different_node =
    different_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_THROW(
    node_topics->add_subscription(subscription, callback_group_in_different_node),
    std::runtime_error);
}

TEST_F(TestNodeTopics, add_subscription_rcl_trigger_guard_condition_error)
{
  auto subscription = std::make_shared<TestSubscription>(node.get());
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_topics->add_subscription(subscription, callback_group),
    std::runtime_error("failed to notify wait set on subscription creation: error not set"));
}

TEST_F(TestNodeTopics, resolve_topic_name)
{
  EXPECT_EQ("/ns/bar", node_topics->resolve_topic_name("foo", false));
  EXPECT_EQ("/ns/foo", node_topics->resolve_topic_name("foo", true));
  EXPECT_EQ("/foo", node_topics->resolve_topic_name("/foo", true));
  EXPECT_THROW(
    node_topics->resolve_topic_name("this is not a valid name!~>", true),
    rclcpp::exceptions::RCLError);
}
