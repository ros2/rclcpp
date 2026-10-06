// Copyright 2026 Fly4Future
// Copyright 2026 Czech Technical University in Prague
// Copyright 2026 Vojtech Spurny
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
#include <future>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"

using namespace std::chrono_literals;

/*
 * A timer is removed from the executor while its callback is running, either
 * by removing its node or by shutting down the context. When the callback
 * returns, the executor re-arms the timer and must not access the bookkeeping
 * of the removed timer.
 */
class TestEventsCBGExecutorTimerRemoval : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor = std::make_unique<rclcpp::executors::EventsCBGExecutor>(
      rclcpp::ExecutorOptions(), 4);

    node = std::make_shared<rclcpp::Node>("test_events_cbg_executor_timer_removal");
    timer = node->create_wall_timer(1ms, [this]() {timer_callback();});

    executor->add_node(node);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor.reset();
  }

  /// Blocks the first call until the timer was removed, or for a bounded time.
  void timer_callback()
  {
    calls++;
    if (calls > 1) {
      return;
    }
    callback_entered.set_value();

    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (node_has_executor() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    // give the removal time to release the timer bookkeeping
    std::this_thread::sleep_for(100ms);
    callback_returned = true;
  }

  bool node_has_executor()
  {
    return node->get_node_base_interface()->get_associated_with_executor_atomic().load();
  }

  bool wait_for_callback()
  {
    return callback_entered.get_future().wait_for(5s) == std::future_status::ready;
  }

  std::unique_ptr<rclcpp::executors::EventsCBGExecutor> executor;
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr timer;
  std::thread spin_thread;

  std::promise<void> callback_entered;
  std::atomic_int calls{0};
  std::atomic_bool callback_returned{false};
};

TEST_F(TestEventsCBGExecutorTimerRemoval, context_shutdown_during_spin)
{
  spin_thread = std::thread([this]() {executor->spin();});
  ASSERT_TRUE(wait_for_callback());

  rclcpp::shutdown();
  spin_thread.join();

  EXPECT_TRUE(callback_returned);
  EXPECT_FALSE(node_has_executor());
}

TEST_F(TestEventsCBGExecutorTimerRemoval, context_shutdown_during_spin_some)
{
  spin_thread = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        executor->spin_some();
      }
    });
  ASSERT_TRUE(wait_for_callback());

  rclcpp::shutdown();
  spin_thread.join();

  EXPECT_TRUE(callback_returned);
  EXPECT_FALSE(node_has_executor());
}

TEST_F(TestEventsCBGExecutorTimerRemoval, remove_node_during_spin)
{
  spin_thread = std::thread([this]() {executor->spin();});
  ASSERT_TRUE(wait_for_callback());

  executor->remove_node(node);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!callback_returned && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(callback_returned);

  // the removed timer must not be re-armed
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(calls, 1);
}
