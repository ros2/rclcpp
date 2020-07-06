// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono_literals;

class TestTimerCount : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

constexpr int EXECUTION_COUNT = 5;
constexpr double TIME_ELAPSED = 12.0;
/*
   Test timer wait mutex with multithreaded executor.
   After 5 call
 */

TEST_F(TestTimerCount, timer_call_count_multi_threaded) {
  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("timer_call_count_multi_threaded");

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::Clock system_clock(RCL_STEADY_TIME);
  std::mutex last_mutex;
  rclcpp::Time initial = system_clock.now();

  std::atomic_int timer_count {0};

  auto timer_callback = [&timer_count, &executor, &system_clock, &last_mutex, &initial]() {
      rclcpp::Time now = system_clock.now();
      timer_count++;

      {
        std::lock_guard<std::mutex> lock(last_mutex);
        double diff = std::abs((now - initial).nanoseconds()) / 1.0e9;

        if (diff > TIME_ELAPSED) {
          executor.cancel();
          ASSERT_GT(timer_count, EXECUTION_COUNT);
        }
      }
    };

  auto timer_ = node->create_wall_timer(
    2s, timer_callback, cbg);

  executor.add_node(node);
  executor.spin();
}

/*
   Test timer wait mutex with singlethreaded executor
 */
TEST_F(TestTimerCount, timer_call_count_single_threaded) {
  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("timer_call_count_single_threaded");

  rclcpp::Clock system_clock(RCL_STEADY_TIME);
  std::mutex last_mutex;
  rclcpp::Time initial = system_clock.now();

  std::atomic_int timer_count {0};

  auto timer_callback = [&timer_count, &executor, &system_clock, &last_mutex, &initial]() {
      rclcpp::Time now = system_clock.now();
      timer_count++;

      {
        std::lock_guard<std::mutex> lock(last_mutex);
        double diff = std::abs((now - initial).nanoseconds()) / 1.0e9;

        if (diff > TIME_ELAPSED) {
          executor.cancel();
          ASSERT_GT(timer_count, EXECUTION_COUNT);
        }
      }
    };

  auto timer_ = node->create_wall_timer(
    2s, timer_callback);

  executor.add_node(node);
  executor.spin();
}
