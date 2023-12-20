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

#include "test_msgs/srv/empty.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using rclcpp::experimental::executors::EventsExecutor;
using rclcpp::experimental::executors::PriorityEventsQueue;

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

  int subscriptions_executed = 0;

  bool msg_received_low = false;
  auto sub_low = node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::SensorDataQoS(),
    [&msg_received_low, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_low = true;
      EXPECT_EQ(subscriptions_executed, 2);
      subscriptions_executed++;
    });

  bool msg_received_medium = false;
  auto sub_medium = node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::SensorDataQoS(),
    [&msg_received_medium, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_medium = true;
      EXPECT_EQ(subscriptions_executed, 1);
      subscriptions_executed++;
    });

  bool msg_received_high = false;
  auto sub_high = node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::SensorDataQoS(),
    [&msg_received_high, &subscriptions_executed](test_msgs::msg::Empty::ConstSharedPtr msg)
    {
      (void)msg;
      msg_received_high = true;
      EXPECT_EQ(subscriptions_executed, 0);
      subscriptions_executed++;
    });

  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", rclcpp::SensorDataQoS());

  // Create executor
  auto extract_priority = [](const rclcpp::experimental::executors::ExecutorEvent & event) {
      return event.type == rclcpp::experimental::executors::ExecutorEventType::TIMER_EVENT ?
             0 : 1;
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
    std::this_thread::sleep_for(25ms);
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
