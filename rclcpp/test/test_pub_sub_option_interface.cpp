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

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

/*
   Testing construction and destruction.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  rclcpp::PublisherOptions<> pub_options;
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 5, pub_options);

  rclcpp::SubscriptionOptions<> sub_options;
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic",
    [](std::shared_ptr<test_msgs::msg::Empty> test_msg) {(void) test_msg;},
    5,
    sub_options);
}
