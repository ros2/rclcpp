// Copyright 2024 Cellumation GmbH
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
#include <rcpputils/compile_warnings.hpp>

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

    std::thread wait_thread = std::thread(
      [&clock, &thread_finished]()
      {
        // make sure the thread starts sleeping late
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        clock->sleep_until(clock->now() + std::chrono::seconds(3));
        thread_finished = true;
      });
    RCPPUTILS_DEPRECATION_WARNING_OFF_START
    // notify the clock, that the sleep shall be interrupted
    clock->cancel_sleep_or_wait();
    RCPPUTILS_DEPRECATION_WARNING_OFF_STOP

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

    std::thread wait_thread = std::thread(
      [&clock, &thread_finished]()
      {
        clock->sleep_until(clock->now() + std::chrono::seconds(3));
        thread_finished = true;
      });

    // make sure the thread is already sleeping before we send the cancel
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCPPUTILS_DEPRECATION_WARNING_OFF_START
    // notify the clock, that the sleep shall be interrupted
    clock->cancel_sleep_or_wait();
    RCPPUTILS_DEPRECATION_WARNING_OFF_STOP

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
  Clocks,
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
  rclcpp::TimeSource time_source(node);
  time_source.attachClock(clock);

  EXPECT_TRUE(clock->ros_time_is_active());

  std::atomic_bool thread_finished = false;

  std::thread wait_thread = std::thread(
    [&clock, &thread_finished]()
    {
      // make sure the thread starts sleeping late
      clock->sleep_until(clock->now() + std::chrono::milliseconds(10));
      thread_finished = true;
    });

  // make sure, that the sim time clock does not wakeup, as no clock is provided
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_FALSE(thread_finished);

  RCPPUTILS_DEPRECATION_WARNING_OFF_START
  // notify the clock, that the sleep shall be interrupted
  clock->cancel_sleep_or_wait();
  RCPPUTILS_DEPRECATION_WARNING_OFF_STOP

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

TEST_F(TestClockWakeup, multiple_threads_wait_on_one_clock) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  std::vector<bool> thread_finished(10, false);

  std::vector<std::thread> threads;

  for (size_t nr = 0; nr < thread_finished.size(); nr++) {
    threads.push_back(
      std::thread(
        [&clock, &thread_finished, nr]()
        {
          // make sure the thread starts sleeping late
          clock->sleep_until(clock->now() + std::chrono::seconds(10));
          thread_finished[nr] = true;
        }));
  }

  // wait a bit so all threads can execute the sleep_until
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  for (const bool & finished : thread_finished) {
    EXPECT_FALSE(finished);
  }

  rclcpp::shutdown();

  auto start_time = std::chrono::steady_clock::now();
  auto cur_time = start_time;
  bool threads_finished = false;
  while (!threads_finished && start_time + std::chrono::seconds(1) > cur_time) {
    threads_finished = true;
    for (const bool finished : thread_finished) {
      if (!finished) {
        threads_finished = false;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cur_time = std::chrono::steady_clock::now();
  }

  for (const bool finished : thread_finished) {
    EXPECT_TRUE(finished);
  }

  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_LT(cur_time, start_time + std::chrono::seconds(1));
}
