// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

/*
 * The ready queue grows with the backlog when callbacks are slower than the
 * incoming message rate (#3213). Each readiness notification is expanded into
 * `nr_msg` ready entities in first_in_first_out_scheduler.cpp, so the queue
 * tracks the backlog and not the number of subscriptions.
 *
 * Both cases assert only what holds today, so they keep passing if a max_events
 * policy is added later.
 */

class TestEventsCBGExecutorBacklog : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

/*
 * A subscriber whose callback is slower than the publish rate accumulates a
 * backlog. The executor must remain correct: every message that is taken must
 * be delivered in publish order, and the run must finish.
 *
 * This is the measurement the max_events work needs: it establishes the
 * baseline unbounded behaviour against which a bounded queue is compared.
 */
TEST_F(TestEventsCBGExecutorBacklog, slow_subscriber_keeps_order_while_backlog_grows)
{
  auto node = std::make_shared<rclcpp::Node>("backlog_order");

  constexpr size_t kMessages = 200;
  constexpr auto kCallbackCost = 5ms;

  std::mutex received_mutex;
  std::vector<int32_t> received;
  received.reserve(kMessages);

  auto subscription = node->create_subscription<std_msgs::msg::Int32>(
    "backlog", rclcpp::QoS(1000),
    [&](std_msgs::msg::Int32::ConstSharedPtr msg) {
      std::this_thread::sleep_for(kCallbackCost);
      std::lock_guard<std::mutex> lock(received_mutex);
      received.push_back(msg->data);
    });

  auto publisher_node = std::make_shared<rclcpp::Node>("backlog_order_pub");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::Int32>(
    "backlog", rclcpp::QoS(1000));

  auto executor = std::make_unique<rclcpp::executors::EventsCBGExecutor>(
    rclcpp::ExecutorOptions(), 1);
  executor->add_node(node);

  std::thread spin_thread([&executor]() {executor->spin();});

  for (size_t i = 0; i < kMessages; ++i) {
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int32_t>(i);
    publisher->publish(msg);
    std::this_thread::sleep_for(1ms);
  }

  const auto deadline = std::chrono::steady_clock::now() + 60s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      if (received.size() >= kMessages) {break;}
    }
    std::this_thread::sleep_for(10ms);
  }

  executor->cancel();
  spin_thread.join();

  std::lock_guard<std::mutex> lock(received_mutex);
  ASSERT_EQ(received.size(), kMessages);

  // Strictly increasing: the executor may not reorder messages of one entity.
  for (size_t i = 0; i < received.size(); ++i) {
    EXPECT_EQ(received[i], static_cast<int32_t>(i))
      << "message " << i << " was delivered out of order";
  }
}

/*
 * The growth path from the issue: one readiness notification carrying `nr_msg`
 * cheaper than the middleware can hand them over. When the callback is slower
 * than the publish rate, the executor holds the whole backlog, and draining it
 * takes time proportional to the backlog.
 *
 * There is no public accessor for the queue, so this measures the time it takes
 * to drain instead: a bound either still delivers all 500 (deferred admission)
 * or drops some, and dropping has to be visible here.
 */
TEST_F(TestEventsCBGExecutorBacklog, slow_callback_absorbs_the_whole_backlog)
{
  auto node = std::make_shared<rclcpp::Node>("backlog_absorb");

  constexpr size_t kMessages = 500;
  constexpr auto kCallbackCost = 2ms;

  std::atomic<size_t> delivered{0};

  auto subscription = node->create_subscription<std_msgs::msg::Int32>(
    "backlog_absorb", rclcpp::QoS(1000),
    [&](std_msgs::msg::Int32::ConstSharedPtr) {
      std::this_thread::sleep_for(kCallbackCost);
      ++delivered;
    });

  auto publisher_node = std::make_shared<rclcpp::Node>("backlog_absorb_pub");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::Int32>(
    "backlog_absorb", rclcpp::QoS(1000));

  auto executor = std::make_unique<rclcpp::executors::EventsCBGExecutor>(
    rclcpp::ExecutorOptions(), 1);
  executor->add_node(node);

  std::thread spin_thread([&executor]() {executor->spin();});
  std::this_thread::sleep_for(500ms);  // let discovery settle, executor idle

  // A burst: the executor can service at most one callback while these are
  // being published, so the rest becomes backlog.
  for (size_t i = 0; i < kMessages; ++i) {
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int32_t>(i);
    publisher->publish(msg);
  }

  const auto drain_start = std::chrono::steady_clock::now();
  const auto deadline = drain_start + 120s;
  while (std::chrono::steady_clock::now() < deadline && delivered.load() < kMessages) {
    std::this_thread::sleep_for(1ms);
  }
  const auto drained_in = std::chrono::steady_clock::now() - drain_start;

  executor->cancel();
  spin_thread.join();

  // Unbounded today: nothing is dropped and nothing is delivered twice.
  EXPECT_EQ(delivered.load(), kMessages)
    << "the executor dropped messages from a " << kMessages << "-message backlog";

  // A backlog really existed: 500 messages at 2 ms each cannot drain in less
  // than a quarter of that. Without this the test would pass by never building
  // one -- an earlier version did exactly that in 16 ms.
  const auto floor = kCallbackCost * (kMessages / 4);
  EXPECT_GT(drained_in, floor)
    << "drained in " << std::chrono::duration_cast<std::chrono::milliseconds>(drained_in).count()
    << " ms, which is too fast to have been a real backlog";
}
