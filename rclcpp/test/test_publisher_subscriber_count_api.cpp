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
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

class MyPublisher : public rclcpp::PublisherBase
{
public:
  size_t get_intraprocess_subscription_count()
  {
    if (get_intra_process_subscription_count_) {
      return get_intra_process_subscription_count_(intra_process_publisher_id_);
    }
    return 0;
  }
};

class TestPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>(
      "my_node",
      "/ns",
      rclcpp::NodeOptions().use_intra_process_comms(true));
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
{
  (void)msg;
}

/*
   Testing publisher subscriber count api, and internal process subscribers.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  using rcl_interfaces::msg::IntraProcessMessage;

  auto publisher = node->create_publisher<IntraProcessMessage>("/topic");
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(
    reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(), 0u);

  auto callback = &OnMessage;
  auto sub = node->create_subscription<IntraProcessMessage>("/topic", callback);
  EXPECT_EQ(publisher->get_subscription_count(), 1u);
  EXPECT_EQ(
    reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(), 1u);

  rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>("another_node", "/ns");
  auto another_sub = another_node->create_subscription<IntraProcessMessage>("/topic", callback);
  EXPECT_EQ(publisher->get_subscription_count(), 2u);
  EXPECT_EQ(
    reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(), 1u);
}
