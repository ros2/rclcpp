// Copyright 2023 Washington University in St. Louis.
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
#include <string>

#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/experimental/executors/events_executor/priority_events_queue.hpp"
// #include "rclcpp/experimental/executors/events_executor/simple_events_queue.hpp"

#include "test_msgs/srv/empty.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using rclcpp::experimental::executors::EventsExecutor;
using rclcpp::experimental::executors::PriorityEventsQueue;
// using rclcpp::experimental::executors::SimpleEventsQueue;

class TestPriorityEventsExecutor : public ::testing::Test
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

TEST_F(TestPriorityEventsExecutor, priority_subs)
{
  // rmw_connextdds doesn't support events-executor
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0) {
    GTEST_SKIP();
  }

  // Create timer publishing to 3 subscriptions
  auto node = std::make_shared<rclcpp::Node>("node");

  // Create QoS for subscriptions
  rclcpp::QoS qos(10);

  int subscriptions_executed = 0;

  qos.deadline(rclcpp::Duration(3, 0));
  bool msg_received_low = false;
  auto sub_low = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [&msg_received_low, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_low = true;
      EXPECT_EQ(subscriptions_executed, 2);
      subscriptions_executed++;
    });

  qos.deadline(rclcpp::Duration(2, 0));
  bool msg_received_medium = false;
  auto sub_medium = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [&msg_received_medium, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_medium = true;
      EXPECT_EQ(subscriptions_executed, 1);
      subscriptions_executed++;
    });

  qos.deadline(rclcpp::Duration(1, 0));
  bool msg_received_high = false;
  auto sub_high = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [&msg_received_high, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_high = true;
      EXPECT_EQ(subscriptions_executed, 0);
      subscriptions_executed++;
    });

  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", qos);

  // Create executor
  auto extract_priority = [](const rclcpp::experimental::executors::ExecutorEvent & event) {
      const rcl_client_t * client;
      const rcl_service_t * service;
      const rclcpp::TimerBase * timer;
      const rclcpp::Waitable * waitable;
      const rcl_subscription_t * subscription;
      switch (event.type) {
        case rclcpp::experimental::executors::ExecutorEventType::CLIENT_EVENT:
          client = static_cast<const rcl_client_t *>(event.entity_key);
          break;
        case rclcpp::experimental::executors::ExecutorEventType::SERVICE_EVENT:
          service = static_cast<const rcl_service_t *>(event.entity_key);
          break;
        case rclcpp::experimental::executors::ExecutorEventType::TIMER_EVENT:
          timer = static_cast<const rclcpp::TimerBase *>(event.entity_key);
          return 0UL;
          break;
        case rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT:
          subscription = static_cast<const rcl_subscription_t *>(event.entity_key);
          return rcl_subscription_get_options(subscription)->qos.deadline.sec;
          break;
        case rclcpp::experimental::executors::ExecutorEventType::WAITABLE_EVENT:
          waitable = static_cast<const rclcpp::Waitable *>(event.entity_key);
          break;
      }
      return 0UL;
    };
  EventsExecutor executor(std::make_unique<PriorityEventsQueue>(extract_priority));
  executor.add_node(node);


  bool spin_exited = false;
  std::thread spinner([&spin_exited, &executor, this]() {
      executor.spin();
      spin_exited = true;
    });

  auto msg = std::make_unique<test_msgs::msg::Empty>();
  publisher->publish(std::move(msg));

  // Wait some time for the subscription to receive the message
  auto start = std::chrono::high_resolution_clock::now();
  while (
    !msg_received_low &&
    !spin_exited &&
    (std::chrono::high_resolution_clock::now() - start < 1s))
  {
    auto time = std::chrono::high_resolution_clock::now() - start;
    auto time_msec = std::chrono::duration_cast<std::chrono::milliseconds>(time);
    std::this_thread::sleep_for(100ms);
  }

  executor.cancel();
  spinner.join();
  executor.remove_node(node);

  EXPECT_TRUE(msg_received_low);
  EXPECT_TRUE(msg_received_medium);
  EXPECT_TRUE(msg_received_high);
  EXPECT_EQ(subscriptions_executed, 3);
  EXPECT_TRUE(spin_exited);
}
