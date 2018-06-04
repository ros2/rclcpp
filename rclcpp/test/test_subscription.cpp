// Copyright 2017 Open Source Robotics Foundation, Inc.
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

class TestSubscription : public ::testing::Test
{
public:
  void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
  {
    (void)msg;
  }

protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_subscription", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class SubscriptionClassNodeInheritance : public rclcpp::Node
{
public:
  SubscriptionClassNodeInheritance()
  : Node("subscription_class_node_inheritance")
  {
  }

  void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
  {
    (void)msg;
  }

  void CreateSubscription()
  {
    auto callback = std::bind(
      &SubscriptionClassNodeInheritance::OnMessage, this, std::placeholders::_1);
    using rcl_interfaces::msg::IntraProcessMessage;
    auto sub = this->create_subscription<IntraProcessMessage>("topic", callback);
  }
};

class SubscriptionClass
{
public:
  void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
  {
    (void)msg;
  }

  void CreateSubscription()
  {
    auto node = std::make_shared<rclcpp::Node>("test_subscription_member_callback", "/ns");
    auto callback = std::bind(&SubscriptionClass::OnMessage, this, std::placeholders::_1);
    using rcl_interfaces::msg::IntraProcessMessage;
    auto sub = node->create_subscription<IntraProcessMessage>("topic", callback);
  }
};

/*
   Testing subscription construction and destruction.
 */
TEST_F(TestSubscription, construction_and_destruction) {
  using rcl_interfaces::msg::IntraProcessMessage;
  auto callback = [](const IntraProcessMessage::SharedPtr msg) {
      (void)msg;
    };
  {
    auto sub = node->create_subscription<IntraProcessMessage>("topic", callback);
  }

  {
    ASSERT_THROW({
      auto sub = node->create_subscription<IntraProcessMessage>("invalid_topic?", callback);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing subscriptions using std::bind.
 */
TEST_F(TestSubscription, callback_bind) {
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    // Member callback for plain class
    SubscriptionClass subscriptionObject;
    subscriptionObject.CreateSubscription();
  }
  {
    // Member callback for class inheriting from rclcpp::Node
    SubscriptionClassNodeInheritance subscriptionObject;
    subscriptionObject.CreateSubscription();
  }
  {
    // Member callback for class inheriting from testing::Test
    // Regression test for https://github.com/ros2/rclcpp/issues/479 where the TEST_F GTest macro
    // was interfering with rclcpp's `function_traits`.
    auto callback = std::bind(&TestSubscription::OnMessage, this, std::placeholders::_1);
    auto sub = node->create_subscription<IntraProcessMessage>("topic", callback);
  }
}
