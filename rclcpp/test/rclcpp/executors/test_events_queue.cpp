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
#include <chrono>
#include <memory>
#include <gtest/gtest.h>

#include "rclcpp/executors/events_executor.hpp"
#include "rclcpp/experimental/buffers/bounded_events_queue.hpp"

#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using rclcpp::executors::EventsExecutor;

class TestEventsQueue : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestEventsQueue, BoundedQueue)
{
  // Create BoundedEventsQueue and set limit to 10 events.
  auto bounded_queue = std::make_unique<rclcpp::experimental::buffers::BoundedEventsQueue>();
  bounded_queue->set_queue_size_limit(10);

  // Create an events executor using the bounded queue
  EventsExecutor executor_sub(std::move(bounded_queue));

  // Create a subscription node
  auto node_sub = std::make_shared<rclcpp::Node>("node_sub");

  size_t callback_count = 0;

  auto subscription =
    node_sub->create_subscription<test_msgs::msg::Empty>(
      "topic",
      rclcpp::QoS(10),
      [&](test_msgs::msg::Empty::SharedPtr) {
        callback_count++;
      });

  // Add susbscription node to the executor, so the queue can start getting events
  executor_sub.add_node(node_sub);

  // Create a publisher node
  auto node_pub = std::make_shared<rclcpp::Node>("node_pub");
  auto publisher = node_pub->create_publisher<test_msgs::msg::Empty>("topic", rclcpp::QoS(10));

  // Let subscriptions executor spin to execute any previous events
  // not related to the subscription, so we start with an empty queue
  executor_sub.spin_some(10ms);

  // Publish 11 messages, the eleventh msg should prune the queue
  // and we should end up with only one event on it
  for (int i = 0; i < 11; i++) {
    publisher->publish(std::make_unique<test_msgs::msg::Empty>());
    std::this_thread::sleep_for(1ms);
  }

  // Let subscriptions executor spin
  executor_sub.spin_some(10ms);

  // The callback count should be 1
  EXPECT_EQ(1u, callback_count);

  // Reset callback count
  callback_count = 0;

  // Publish 5 messages
  for (int i = 0; i < 5; i++) {
    publisher->publish(std::make_unique<test_msgs::msg::Empty>());
    std::this_thread::sleep_for(1ms);
  }

  // Let subscriptions executor spin
  executor_sub.spin_some(10ms);

  // The callback count should be 5, the queue shouldn't have been pruned
  EXPECT_EQ(5u, callback_count);
}

TEST_F(TestEventsQueue, SimpleQueue)
{
  // Create SimpleEventsQueue
  auto simple_queue = std::make_unique<rclcpp::experimental::buffers::SimpleEventsQueue>();

  // Create an events executor using the simple queue
  EventsExecutor executor_sub(std::move(simple_queue));

  // Create a subscription node with QoS->depth = 10
  auto node_sub = std::make_shared<rclcpp::Node>("node_sub");

  size_t callback_count = 0;

  auto subscription =
    node_sub->create_subscription<test_msgs::msg::Empty>(
      "topic",
      rclcpp::QoS(10),
      [&](test_msgs::msg::Empty::SharedPtr) {
        callback_count++;
      });

  // Add susbscription node to the executor, so the queue can start getting events
  executor_sub.add_node(node_sub);

  // Create a publisher node
  auto node_pub = std::make_shared<rclcpp::Node>("node_pub");
  auto publisher = node_pub->create_publisher<test_msgs::msg::Empty>("topic", rclcpp::QoS(10));

  // Let subscriptions executor spin to execute any previous events
  // not related to the subscription, so we start with an empty queue
  executor_sub.spin_some(10ms);

  // Publish 11 messages, but as the subscription has only 10 as queue depth,
  // we should lost a message (the events queue finish with 11 messages but one is not valid)
  for (int i = 0; i < 11; i++) {
    publisher->publish(std::make_unique<test_msgs::msg::Empty>());
    std::this_thread::sleep_for(1ms);
  }

  // Let subscriptions executor spin
  executor_sub.spin_some(10ms);

  // The callback count should be 10
  EXPECT_EQ(10u, callback_count);
}
