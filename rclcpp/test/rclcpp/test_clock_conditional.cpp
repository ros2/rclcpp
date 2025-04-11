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

#include <chrono>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "../utils/rclcpp_gtest_macros.hpp"

using namespace std::chrono_literals;

class TestClockWakeup : public ::testing::TestWithParam<rcl_clock_type_e>
{
public:
  void test_wakeup_before_sleep(const rclcpp::Clock::SharedPtr & clock)
  {
    std::atomic_bool thread_finished = false;
    rclcpp::ClockConditionalVariable cond(clock);

    bool stopSleeping = false;

    std::thread wait_thread = std::thread(
      [&cond, &clock, &stopSleeping, &thread_finished]()
      {
        // make sure the thread starts sleeping late
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock lk(cond.mutex());
        cond.wait_until(lk, clock->now() + std::chrono::seconds(3),
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
    rclcpp::ClockConditionalVariable cond(clock);

    bool stopSleeping = false;

    std::thread wait_thread = std::thread(
      [&cond, &clock, &stopSleeping, &thread_finished]()
      {
        std::unique_lock lk(cond.mutex());
        cond.wait_until(lk, clock->now() + std::chrono::seconds(3),
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

TEST_F(TestClockWakeup, wakeup_sleep_ros_time_active) {
  node->set_parameter({"use_sim_time", true});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source(node);
  time_source.attachClock(clock);

  EXPECT_TRUE(clock->ros_time_is_active());

  test_wakeup_after_sleep(clock);
  test_wakeup_before_sleep(clock);
}

TEST_F(TestClockWakeup, no_wakeup_on_sim_time) {
  node->set_parameter({"use_sim_time", true});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_FALSE(clock->ros_time_is_active());

  rclcpp::TimeSource time_source(node);
  time_source.attachClock(clock);
  EXPECT_TRUE(clock->ros_time_is_active());

  std::atomic_bool thread_finished = false;
  rclcpp::ClockConditionalVariable cond(clock);

  bool stopSleeping = false;

  std::thread wait_thread = std::thread(
    [&cond, &clock, &stopSleeping, &thread_finished]()
    {
      std::unique_lock lk(cond.mutex());
      // only sleep for an short period
      cond.wait_until(lk, clock->now() + std::chrono::milliseconds(10),
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
