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

#include "rclcpp/executors/events_executor.hpp"

#include "test_msgs/srv/empty.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using rclcpp::executors::EventsExecutor;

class TestEventsExecutor : public ::testing::Test
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

TEST_F(TestEventsExecutor, run_clients_servers)
{
  auto node = std::make_shared<rclcpp::Node>("node");

  bool request_received = false;
  bool response_received = false;
  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [&request_received](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr)
    {
      request_received = true;
    });
  auto client = node->create_client<test_msgs::srv::Empty>("service");

  EventsExecutor executor;
  executor.add_node(node);

  bool spin_exited = false;
  std::thread spinner([&spin_exited, &executor, this]() {
      executor.spin();
      spin_exited = true;
    });

  auto request = std::make_shared<test_msgs::srv::Empty::Request>();
  client->async_send_request(
    request,
    [&response_received](rclcpp::Client<test_msgs::srv::Empty>::SharedFuture result_future) {
      (void)result_future;
      response_received = true;
    });

  // Wait some time for the client-server to be invoked
  auto start = std::chrono::steady_clock::now();
  while (
    !response_received &&
    !spin_exited &&
    (std::chrono::steady_clock::now() - start < 1s))
  {
    std::this_thread::sleep_for(5ms);
  }

  executor.cancel();
  spinner.join();
  executor.remove_node(node);

  EXPECT_TRUE(request_received);
  EXPECT_TRUE(response_received);
  EXPECT_TRUE(spin_exited);
}

TEST_F(TestEventsExecutor, spin_once_max_duration)
{
  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10s,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_once(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }

  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10ms,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_once(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }
}

TEST_F(TestEventsExecutor, spin_some_max_duration)
{
  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10s,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_some(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }

  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10ms,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_some(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }
}

TEST_F(TestEventsExecutor, spin_all_max_duration)
{
  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10s,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_all(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }

  {
    auto node = std::make_shared<rclcpp::Node>("node");

    size_t t_runs = 0;
    auto t = node->create_wall_timer(
      10ms,
      [&]() {
        t_runs++;
      });

    EventsExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();
    executor.spin_all(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(std::chrono::steady_clock::now() - start < 200ms);
  }

  EventsExecutor executor;
  EXPECT_THROW(executor.spin_all(0ms), std::invalid_argument);
  EXPECT_THROW(executor.spin_all(-5ms), std::invalid_argument);
}

TEST_F(TestEventsExecutor, cancel_while_timers_waiting)
{
  auto node = std::make_shared<rclcpp::Node>("node");

  size_t t1_runs = 0;
  auto t1 = node->create_wall_timer(
    100s,
    [&]() {
      t1_runs++;
    });

  EventsExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::steady_clock::now();
  std::thread spinner([&executor, this]() {executor.spin();});

  std::this_thread::sleep_for(10ms);
  executor.cancel();
  spinner.join();

  EXPECT_EQ(0u, t1_runs);
  EXPECT_TRUE(std::chrono::steady_clock::now() - start < 1s);
}

TEST_F(TestEventsExecutor, destroy_entities)
{
  // Create a publisher node and start publishing messages
  auto node_pub = std::make_shared<rclcpp::Node>("node_pub");
  auto publisher = node_pub->create_publisher<test_msgs::msg::Empty>("topic", rclcpp::QoS(10));
  auto timer = node_pub->create_wall_timer(
    2ms, [&]() { publisher->publish(std::make_unique<test_msgs::msg::Empty>()); });
  EventsExecutor executor_pub;
  executor_pub.add_node(node_pub);
  std::thread spinner([&executor_pub, this]() {executor_pub.spin();});

  // Create a node with two different subscriptions to the topic
  auto node_sub = std::make_shared<rclcpp::Node>("node_sub");
  size_t callback_count_1 = 0;
  auto subscription_1 =
    node_sub->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), [&](test_msgs::msg::Empty::SharedPtr) {callback_count_1++;});
  size_t callback_count_2 = 0;
  auto subscription_2 =
    node_sub->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), [&](test_msgs::msg::Empty::SharedPtr) {callback_count_2++;});
  EventsExecutor executor_sub;
  executor_sub.add_node(node_sub);

  // Wait some time while messages are published
  std::this_thread::sleep_for(10ms);

  // Destroy one of the two subscriptions
  subscription_1.reset();

  // Let subscriptions executor spin
  executor_sub.spin_some(10ms);

  // The callback count of the destroyed subscription remained at 0
  EXPECT_EQ(0u, callback_count_1);
  EXPECT_LT(0u, callback_count_2);

  executor_pub.cancel();
  spinner.join();
}
