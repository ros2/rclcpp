// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class TestMultiThreadedExecutor : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

constexpr std::chrono::milliseconds PERIOD_MS = 1000ms;
constexpr double PERIOD = PERIOD_MS.count() / 1000.0;
constexpr double TOLERANCE = PERIOD / 4.0;

/*
   Test that timers are not taken multiple times when using reentrant callback groups.
 */
TEST_F(TestMultiThreadedExecutor, timer_over_take) {
#ifdef __linux__
  // This seems to be the most effective way to force the bug to happen on Linux.
  // This is unnecessary on MacOS, since the default scheduler causes it.
  struct sched_param param;
  param.sched_priority = 0;
  if (sched_setscheduler(0, SCHED_BATCH, &param) != 0) {
    perror("sched_setscheduler");
  }
#endif

  bool yield_before_execute = true;

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2u, yield_before_execute);

  ASSERT_GT(executor.get_number_of_threads(), 1u);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_multi_threaded_executor_timer_over_take");

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::Clock system_clock(RCL_STEADY_TIME);
  std::mutex last_mutex;
  auto last = system_clock.now();

  std::atomic_int timer_count {0};

  auto timer_callback = [&timer_count, &executor, &system_clock, &last_mutex, &last]() {
      // While this tolerance is a little wide, if the bug occurs, the next step will
      // happen almost instantly. The purpose of this test is not to measure the jitter
      // in timers, just assert that a reasonable amount of time has passed.
      rclcpp::Time now = system_clock.now();
      timer_count++;

      if (timer_count > 5) {
        executor.cancel();
      }

      {
        std::lock_guard<std::mutex> lock(last_mutex);
        double diff = static_cast<double>(std::abs((now - last).nanoseconds())) / 1.0e9;
        last = now;

        if (diff < PERIOD - TOLERANCE) {
          executor.cancel();
          ASSERT_GT(diff, PERIOD - TOLERANCE);
        }
      }
    };

  auto timer = node->create_wall_timer(PERIOD_MS, timer_callback, cbg);
  executor.add_node(node);
  executor.spin();
}

/*
   Test that no tasks are starved
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 104b5d5b (Updated test for multi-threaded executor starvation)
/**
 * @brief Test case for checking starvation in MultiThreadedExecutor.
 *
 * This test creates a MultiThreadedExecutor with two threads and a node with
 * two timers. Each timer has a callback that increments a counter and checks
 * the difference between the two counters. If the difference exceeds 1, the
 * test will shut down the executor and assert that the difference is less than
 * or equal to 1.
 *
 * The purpose of this test is to ensure that the MultiThreadedExecutor does not
 * starve any of the timers and that both timers are executed in a balanced
 * manner.
 */
<<<<<<< HEAD
TEST_F(TestMultiThreadedExecutor, starvation) {
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
    2u);
  // Create a node for the test
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_multi_threaded_executor_starvation");

  // Atomic counters for the timers
  std::atomic_int timer_one_count{0};
  std::atomic_int timer_two_count{0};

  // Callback for the timers
  auto timer_one_callback = [](std::atomic_int & count_one,
    std::atomic_int & count_two) -> void {
      std::cout << "Timer one callback executed. Count: " << count_one.load()
                << std::endl;

    // Simulate work by busy-waiting for 100ms
      auto start_time = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start_time < 100ms) {
      }

    // Increment the counter for the first timer
      count_one++;

    // Calculate the difference between the two counters
      auto diff = std::abs(count_one - count_two);

      std::cout << "Difference in counts: " << diff << std::endl;

    // If the difference exceeds 1, shut down and assert failure
      if (diff > 1) {
        rclcpp::shutdown();
        ASSERT_LE(diff, 1);
      }
    };

  // Create the timers with 0ms period
  auto timer_one = node->create_wall_timer(
      0ms, [timer_one_callback, &timer_one_count, &timer_two_count]() {
      timer_one_callback(timer_one_count, timer_two_count);
      });
  auto timer_two = node->create_wall_timer(
      0ms, [timer_one_callback, &timer_two_count, &timer_one_count]() {
      timer_one_callback(timer_two_count, timer_one_count);
      });

  // Add the node to the executor and spin
  executor.add_node(node);
  executor.spin();
}
=======
=======
>>>>>>> 104b5d5b (Updated test for multi-threaded executor starvation)
TEST_F(TestMultiThreadedExecutor, starvation) {
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
    2u);
  // Create a node for the test
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_multi_threaded_executor_starvation");

  // Atomic counters for the timers
  std::atomic_int timer_one_count{0};
  std::atomic_int timer_two_count{0};

  // Callback for the timers
  auto timer_one_callback = [](std::atomic_int & count_one,
    std::atomic_int & count_two) -> void {
      std::cout << "Timer one callback executed. Count: " << count_one.load()
                << std::endl;

    // Simulate work by busy-waiting for 100ms
      auto start_time = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start_time < 100ms) {
      }

    // Increment the counter for the first timer
      count_one++;

    // Calculate the difference between the two counters
      auto diff = std::abs(count_one - count_two);

      std::cout << "Difference in counts: " << diff << std::endl;

    // If the difference exceeds 1, shut down and assert failure
      if (diff > 1) {
        rclcpp::shutdown();
        ASSERT_LE(diff, 1);
      }
    };

  // Create the timers with 0ms period
  auto timer_one = node->create_wall_timer(
      0ms, [timer_one_callback, &timer_one_count, &timer_two_count]() {
      timer_one_callback(timer_one_count, timer_two_count);
      });
  auto timer_two = node->create_wall_timer(
      0ms, [timer_one_callback, &timer_two_count, &timer_one_count]() {
      timer_one_callback(timer_two_count, timer_one_count);
      });

  // Add the node to the executor and spin
  executor.add_node(node);
  executor.spin();
}
<<<<<<< HEAD
>>>>>>> b8596345 (Added starvation test)
=======
>>>>>>> 15c6c501 (Refactor multi-threaded executor tests for improved readability and consistency)
