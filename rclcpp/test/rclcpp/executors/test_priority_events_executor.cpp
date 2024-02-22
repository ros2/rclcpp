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

TEST_F(TestPriorityEventsExecutor, priority_queue)
{
  // Make node
  auto node = std::make_shared<rclcpp::Node>("node");

  // Create QoS for subscriptions
  rclcpp::QoS qos(10);

  qos.deadline(rclcpp::Duration(3, 0));
  auto sub_low = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
    });
  rclcpp::experimental::executors::ExecutorEvent event_low = {
    sub_low->get_subscription_handle().get(),
    0,
    rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT,
    1
  };

  qos.deadline(rclcpp::Duration(2, 0));
  auto sub_medium = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
    });
  rclcpp::experimental::executors::ExecutorEvent event_medium = {
    sub_medium->get_subscription_handle().get(),
    0,
    rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT,
    1
  };

  qos.deadline(rclcpp::Duration(1, 0));
  auto sub_high = node->create_subscription<test_msgs::msg::Empty>(
    "topic", qos,
    [](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
    });
  rclcpp::experimental::executors::ExecutorEvent event_high = {
    sub_high->get_subscription_handle().get(),
    0,
    rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT,
    1
  };

  auto tmr = node->create_wall_timer(1s, []() {});
  rclcpp::experimental::executors::ExecutorEvent event_tmr = {
    tmr.get(),
    0,
    rclcpp::experimental::executors::ExecutorEventType::TIMER_EVENT,
    1
  };

  // Create PriorityEventsQueue
  auto extract_priority = [](const rclcpp::experimental::executors::ExecutorEvent & event) {
      if (event.type != rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT) {
        return 0UL;
      }
      auto subscription = static_cast<const rcl_subscription_t *>(event.entity_key);
      return rcl_subscription_get_options(subscription)->qos.deadline.sec;
    };
  PriorityEventsQueue::SharedPtr queue = std::make_unique<PriorityEventsQueue>(extract_priority);

  rclcpp::experimental::executors::ExecutorEvent event;
  EXPECT_EQ(queue->empty(), true);
  EXPECT_EQ(queue->dequeue(event, 0s), false);

  queue->enqueue(event_low);
  queue->enqueue(event_medium);
  queue->enqueue(event_high);
  queue->enqueue(event_tmr);


  EXPECT_EQ(queue->dequeue(event, 0s), true);
  EXPECT_EQ(event.entity_key, tmr.get());
  EXPECT_EQ(event.type, rclcpp::experimental::executors::ExecutorEventType::TIMER_EVENT);
  EXPECT_EQ(queue->dequeue(event, 0s), true);
  EXPECT_EQ(event.entity_key, sub_high->get_subscription_handle().get());
  EXPECT_EQ(event.type, rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT);
  EXPECT_EQ(queue->dequeue(event, 0s), true);
  EXPECT_EQ(event.entity_key, sub_medium->get_subscription_handle().get());
  EXPECT_EQ(event.type, rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT);
  EXPECT_EQ(queue->dequeue(event, 0s), true);
  EXPECT_EQ(event.entity_key, sub_low->get_subscription_handle().get());
  EXPECT_EQ(event.type, rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT);
  EXPECT_EQ(queue->dequeue(event, 0s), false);
}

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
      if (event.type != rclcpp::experimental::executors::ExecutorEventType::SUBSCRIPTION_EVENT) {
        return 0UL;
      }
      auto subscription = static_cast<const rcl_subscription_t *>(event.entity_key);
      return rcl_subscription_get_options(subscription)->qos.deadline.sec;
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
