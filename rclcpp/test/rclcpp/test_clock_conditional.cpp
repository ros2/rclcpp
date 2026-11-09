// Copyright 2025 Cellumation GmbH
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
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/utilities.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

using namespace std::chrono_literals;

class TestClockWakeup : public ::testing::TestWithParam<rcl_clock_type_e>
{
public:
  /// Create a conditional variable bound to the clock, or one that is given the clock per wait
  rclcpp::ClockConditionalVariable::UniquePtr
  make_cond(const rclcpp::Clock::SharedPtr & clock)
  {
    if (clock_on_wait) {
      return std::make_unique<rclcpp::ClockConditionalVariable>(
        rclcpp::contexts::get_global_default_context());
    }
    return std::make_unique<rclcpp::ClockConditionalVariable>(clock);
  }

  void
  wait_until(
    rclcpp::ClockConditionalVariable & cond, std::unique_lock<std::mutex> & lk,
    const rclcpp::Clock::SharedPtr & clock, const rclcpp::Time & until,
    const std::function<bool ()> & pred)
  {
    if (clock_on_wait) {
      cond.wait_until(lk, clock, until, pred);
    } else {
      cond.wait_until(lk, until, pred);
    }
  }

  void test_wakeup_before_sleep(const rclcpp::Clock::SharedPtr & clock)
  {
    std::atomic_bool thread_finished = false;
    auto cond_ptr = make_cond(clock);
    rclcpp::ClockConditionalVariable & cond = *cond_ptr;

    bool stopSleeping = false;

    std::thread wait_thread = std::thread(
      [this, &cond, &clock, &stopSleeping, &thread_finished]()
      {
        // make sure the thread starts sleeping late
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock lk(cond.mutex());
        wait_until(cond, lk, clock, clock->now() + std::chrono::seconds(3),
        [&stopSleeping] () {return stopSleeping;});
        thread_finished = true;
      });

    {
      std::lock_guard lk(cond.mutex());
      // stop sleeping after next notification
      stopSleeping = true;
    }
    // notify the conditional, to recheck it pred
    cond.notify_one();

    auto start_time = std::chrono::steady_clock::now();
    auto cur_time = start_time;
    while (!thread_finished && start_time + std::chrono::seconds(1) > cur_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      cur_time = std::chrono::steady_clock::now();
    }

    wait_thread.join();

    EXPECT_TRUE(thread_finished);
    EXPECT_LT(cur_time, start_time + std::chrono::seconds(1));
  }

  void test_wakeup_after_sleep(const rclcpp::Clock::SharedPtr & clock)
  {
    std::atomic_bool thread_finished = false;
    auto cond_ptr = make_cond(clock);
    rclcpp::ClockConditionalVariable & cond = *cond_ptr;

    bool stopSleeping = false;

    std::thread wait_thread = std::thread(
      [this, &cond, &clock, &stopSleeping, &thread_finished]()
      {
        std::unique_lock lk(cond.mutex());
        wait_until(cond, lk, clock, clock->now() + std::chrono::seconds(3),
        [&stopSleeping] () {return stopSleeping;});
        thread_finished = true;
      });

    // make sure the thread is already sleeping before we send the cancel
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
      std::lock_guard lk(cond.mutex());
      // stop sleeping after next notification
      stopSleeping = true;
    }
    // notify the conditional, to recheck it pred
    cond.notify_one();

    auto start_time = std::chrono::steady_clock::now();
    auto cur_time = start_time;
    while (!thread_finished && start_time + std::chrono::seconds(1) > cur_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      cur_time = std::chrono::steady_clock::now();
    }

    wait_thread.join();

    EXPECT_TRUE(thread_finished);
    EXPECT_LT(cur_time, start_time + std::chrono::seconds(1));
  }

  void test_wakeup_sleep_ros_time_active()
  {
    node->set_parameter({"use_sim_time", true});
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::TimeSource time_source(node);
    time_source.attachClock(clock);

    EXPECT_TRUE(clock->ros_time_is_active());

    test_wakeup_after_sleep(clock);
    test_wakeup_before_sleep(clock);
  }

  void test_no_wakeup_on_sim_time()
  {
    node->set_parameter({"use_sim_time", true});
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    EXPECT_FALSE(clock->ros_time_is_active());

    rclcpp::TimeSource time_source(node);
    time_source.attachClock(clock);
    EXPECT_TRUE(clock->ros_time_is_active());

    std::atomic_bool thread_finished = false;
    auto cond_ptr = make_cond(clock);
    rclcpp::ClockConditionalVariable & cond = *cond_ptr;

    bool stopSleeping = false;

    std::thread wait_thread = std::thread(
      [this, &cond, &clock, &stopSleeping, &thread_finished]()
      {
        std::unique_lock lk(cond.mutex());
        // only sleep for an short period
        wait_until(cond, lk, clock, clock->now() + std::chrono::milliseconds(10),
        [&stopSleeping] () {return stopSleeping;});
        thread_finished = true;
      });

    // make sure, that the sim time clock does not wakeup, as no clock is provided
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_FALSE(thread_finished);

    {
      std::lock_guard lk(cond.mutex());
      // stop sleeping after next notification
      stopSleeping = true;
    }
    // notify the conditional, to recheck it pred
    cond.notify_one();

    auto start_time = std::chrono::steady_clock::now();
    auto cur_time = start_time;
    while (!thread_finished && start_time + std::chrono::seconds(1) > cur_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      cur_time = std::chrono::steady_clock::now();
    }

    wait_thread.join();

    EXPECT_TRUE(thread_finished);
    EXPECT_LT(cur_time, start_time + std::chrono::seconds(1));
  }

protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;

  /// Name the clock on every wait, instead of binding it at construction
  bool clock_on_wait = false;
};

INSTANTIATE_TEST_SUITE_P(
  ClockConditionalVariable,
  TestClockWakeup,
  ::testing::Values(
    RCL_SYSTEM_TIME, RCL_ROS_TIME, RCL_STEADY_TIME
));

TEST_P(TestClockWakeup, wakeup_sleep) {
  auto clock = std::make_shared<rclcpp::Clock>(GetParam());
  test_wakeup_after_sleep(clock);
  test_wakeup_before_sleep(clock);
}

TEST_P(TestClockWakeup, wakeup_sleep_clock_given_on_wait) {
  clock_on_wait = true;
  auto clock = std::make_shared<rclcpp::Clock>(GetParam());
  test_wakeup_after_sleep(clock);
  test_wakeup_before_sleep(clock);
}

TEST_F(TestClockWakeup, wakeup_sleep_ros_time_active) {
  test_wakeup_sleep_ros_time_active();
}

TEST_F(TestClockWakeup, wakeup_sleep_ros_time_active_clock_given_on_wait) {
  clock_on_wait = true;
  test_wakeup_sleep_ros_time_active();
}

TEST_F(TestClockWakeup, no_wakeup_on_sim_time) {
  test_no_wakeup_on_sim_time();
}

TEST_F(TestClockWakeup, no_wakeup_on_sim_time_clock_given_on_wait) {
  clock_on_wait = true;
  test_no_wakeup_on_sim_time();
}

TEST_P(TestClockWakeup, clock_given_on_wait_is_not_retained) {
  auto clock = std::make_shared<rclcpp::Clock>(GetParam());
  rclcpp::ClockConditionalVariable cond(rclcpp::contexts::get_global_default_context());
  const auto use_count = clock.use_count();

  std::unique_lock lk(cond.mutex());
  EXPECT_TRUE(cond.wait_until(lk, clock, clock->now() + 10ms, [] () {return false;}));

  EXPECT_EQ(use_count, clock.use_count());
}

TEST_F(TestClockWakeup, wait_without_clock_throws) {
  rclcpp::ClockConditionalVariable cond(rclcpp::contexts::get_global_default_context());
  std::unique_lock lk(cond.mutex());

  EXPECT_THROW(
    cond.wait_until(lk, rclcpp::Time(0, 0), [] () {return true;}),
    std::runtime_error);
}

TEST_F(TestClockWakeup, wait_with_null_clock_throws) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  rclcpp::ClockConditionalVariable cond(clock);
  std::unique_lock lk(cond.mutex());

  // A null clock is an error, it does not fall back to the clock given at construction
  EXPECT_THROW(
    cond.wait_until(lk, nullptr, clock->now(), [] () {return true;}),
    std::runtime_error);
}

TEST_F(TestClockWakeup, wait_on_system_time_ignores_until_clock_type) {
  // These paths only read until's nanoseconds, so a time of another clock type is accepted
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_FALSE(ros_clock->ros_time_is_active());
  auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const auto soon = [] (const rclcpp::Clock::SharedPtr & clock, rcl_clock_type_t type) {
      return rclcpp::Time((clock->now() + 10ms).nanoseconds(), type);
    };

  rclcpp::ClockConditionalVariable ros_cond(ros_clock);
  std::unique_lock ros_lk(ros_cond.mutex());
  EXPECT_NO_THROW(
    ros_cond.wait_until(ros_lk, soon(ros_clock, RCL_SYSTEM_TIME), [] () {return false;}));

  rclcpp::ClockConditionalVariable system_cond(system_clock);
  std::unique_lock system_lk(system_cond.mutex());
  EXPECT_NO_THROW(
    system_cond.wait_until(system_lk, soon(system_clock, RCL_ROS_TIME), [] () {return false;}));
}

TEST_P(TestClockWakeup, clock_waiter_clock_given_on_wait_times_out) {
  auto clock = std::make_shared<rclcpp::Clock>(GetParam());
  rclcpp::ClockWaiter waiter;
  std::mutex mutex;
  std::unique_lock lk(mutex);

  const auto start_time = std::chrono::steady_clock::now();
  EXPECT_FALSE(waiter.wait_until(lk, clock, clock->now() + 10ms, [] () {return false;}));
  EXPECT_LT(std::chrono::steady_clock::now(), start_time + 1s);
}

TEST_F(TestClockWakeup, clock_waiter_without_clock_throws) {
  rclcpp::ClockWaiter waiter;
  std::mutex mutex;
  std::unique_lock lk(mutex);

  EXPECT_THROW(
    waiter.wait_until(lk, rclcpp::Time(0, 0), [] () {return true;}),
    std::runtime_error);
  EXPECT_THROW(
    waiter.wait_until(lk, nullptr, rclcpp::Time(0, 0), [] () {return true;}),
    std::runtime_error);
}

TEST_F(TestClockWakeup, wait_after_shutdown_returns_false) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  rclcpp::ClockConditionalVariable cond(clock, context);

  context->shutdown("test done");

  std::unique_lock lk(cond.mutex());
  EXPECT_FALSE(
    cond.wait_until(lk, clock->now() + std::chrono::seconds(10), [] () {return false;}));
}

TEST_F(TestClockWakeup, wakeup_on_ros_shutdown) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  std::atomic_bool thread_finished = false;
  rclcpp::ClockConditionalVariable cond(clock);

  bool stopSleeping = false;

  std::thread wait_thread = std::thread(
    [&cond, &clock, &stopSleeping, &thread_finished]()
    {
      std::unique_lock lk(cond.mutex());
      // only sleep for an short period
      cond.wait_until(lk, clock->now() + std::chrono::seconds(10),
      [&stopSleeping] () {return stopSleeping;});
      thread_finished = true;
    });

  // wait a bit to be sure the thread is sleeping
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_FALSE(thread_finished);

  rclcpp::shutdown();

  auto start_time = std::chrono::steady_clock::now();
  auto cur_time = start_time;
  while (!thread_finished && start_time + std::chrono::seconds(1) > cur_time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cur_time = std::chrono::steady_clock::now();
  }

  EXPECT_TRUE(thread_finished);

  wait_thread.join();

  EXPECT_LT(cur_time, start_time + std::chrono::seconds(1));
}
