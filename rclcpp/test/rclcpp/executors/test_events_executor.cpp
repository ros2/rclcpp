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

    auto start = node->now();
    executor.spin_once(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
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

    auto start = node->now();
    executor.spin_once(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
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

    auto start = node->now();
    executor.spin_some(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
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

    auto start = node->now();
    executor.spin_some(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
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

    auto start = node->now();
    executor.spin_all(10ms);

    EXPECT_EQ(0u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
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

    auto start = node->now();
    executor.spin_all(10s);

    EXPECT_EQ(1u, t_runs);
    EXPECT_TRUE(node->now() - start < 200ms);
  }

  EventsExecutor executor;
  EXPECT_THROW(executor.spin_all(0ms), std::invalid_argument);
  EXPECT_THROW(executor.spin_all(-5ms), std::invalid_argument);
}

TEST_F(TestEventsExecutor, cancel_while_timers_running)
{
  auto node = std::make_shared<rclcpp::Node>("node");

  size_t t1_runs = 0;
  auto t1 = node->create_wall_timer(
    1ms,
    [&]() {
      t1_runs++;
      std::this_thread::sleep_for(25ms);
    });

  size_t t2_runs = 0;
  auto t2 = node->create_wall_timer(
    1ms,
    [&]() {
      t2_runs++;
      std::this_thread::sleep_for(25ms);
    });

  EventsExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor, this]() {executor.spin();});

  std::this_thread::sleep_for(10ms);
  // Call cancel while t1 callback is still being executed
  executor.cancel();
  spinner.join();

  // Depending on the latency on the system, t2 may start to execute before cancel is signaled
  EXPECT_GE(1u, t1_runs);
  EXPECT_GE(1u, t2_runs);
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

  std::thread spinner([&executor, this]() {executor.spin();});

  std::this_thread::sleep_for(10ms);
  executor.cancel();
  spinner.join();

  EXPECT_EQ(0u, t1_runs);
}
