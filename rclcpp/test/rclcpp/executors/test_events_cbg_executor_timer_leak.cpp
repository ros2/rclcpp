// Copyright 2026 Preferred Robotics, Inc.
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

using namespace std::chrono_literals;

class TestEventsCBGExecutorTimerLeak : public testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_events_cbg_executor_timer_leak");
    executor = std::make_unique<rclcpp::executors::EventsCBGExecutor>(
      rclcpp::ExecutorOptions(), 1u);
    executor->add_node(node);
    spin_thread = std::thread([this]() {executor->spin();});
  }

  void TearDown() override
  {
    executor->cancel();
    spin_thread.join();
    other_timers.clear();
    executor.reset();
    node.reset();
  }

  // The executor registers timers in its timer queue when it picks up a new timer
  void add_another_timer()
  {
    other_timers.push_back(node->create_wall_timer(1h, []() {}));
  }

  // The rcl handle stays alive as long as the timer queue holds the timer
  bool wait_until_released(
    const std::weak_ptr<const rcl_timer_t> & handle,
    std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      add_another_timer();
      std::this_thread::sleep_for(50ms);
      if (handle.expired()) {
        return true;
      }
    }
    return false;
  }

  template<typename PredT>
  static bool wait_for(PredT pred, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::EventsCBGExecutor> executor;
  std::thread spin_thread;
  std::vector<rclcpp::TimerBase::SharedPtr> other_timers;
};

/*
 * A one shot timer that cancels itself from its callback and is dropped
 * afterwards must be released by the executor.
 */
TEST_F(TestEventsCBGExecutorTimerLeak, timer_canceled_in_callback_is_released_after_drop)
{
  std::atomic_int fired{0};
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    10ms, [&]() {
      timer->cancel();
      fired++;
    });
  std::weak_ptr<const rcl_timer_t> handle = timer->get_timer_handle();

  ASSERT_TRUE(wait_for([&]() {return fired.load() == 1;}));
  // let the executor finish the timer execution
  std::this_thread::sleep_for(100ms);

  timer.reset();
  EXPECT_TRUE(wait_until_released(handle));
}

/*
 * A timer canceled before it was due, and dropped after its due time passed,
 * must be released by the executor.
 */
TEST_F(TestEventsCBGExecutorTimerLeak, timer_canceled_before_due_is_released_after_drop)
{
  std::atomic_int fired{0};
  auto timer = node->create_wall_timer(100ms, [&]() {fired++;});
  std::weak_ptr<const rcl_timer_t> handle = timer->get_timer_handle();

  // wait until the executor registered the timer
  std::this_thread::sleep_for(50ms);
  timer->cancel();
  // the timer queue drops the canceled timer from its running list at the due time
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(fired.load(), 0);

  timer.reset();
  EXPECT_TRUE(wait_until_released(handle));
}

/*
 * A canceled timer that is still owned by the user must keep working after reset.
 */
TEST_F(TestEventsCBGExecutorTimerLeak, canceled_timer_still_owned_fires_after_reset)
{
  std::atomic_int fired{0};
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    10ms, [&]() {
      timer->cancel();
      fired++;
    });

  ASSERT_TRUE(wait_for([&]() {return fired.load() == 1;}));
  std::this_thread::sleep_for(100ms);

  // other timers make the executor clean up its timer queue
  add_another_timer();
  std::this_thread::sleep_for(100ms);

  timer->reset();
  EXPECT_TRUE(wait_for([&]() {return fired.load() == 2;}));
}
