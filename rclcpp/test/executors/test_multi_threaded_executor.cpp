// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rcl_interfaces/msg/intra_process_message.hpp"

using namespace std::chrono_literals;

using rcl_interfaces::msg::IntraProcessMessage;

class TestMultiThreadedExecutor : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
   Test that timers are not taken multiple times when using reentrant callback groups.
 */
TEST_F(TestMultiThreadedExecutor, timer_over_take) {
  struct sched_param param;
  param.sched_priority = 0;
  if (sched_setscheduler(0, SCHED_BATCH, &param) != 0) {
    perror("sched_setscheduler");
  }

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::create_default_executor_arguments(), 0, true);

  ASSERT_GT(executor.get_number_of_threads(), 1u);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_multi_threaded_executor_timer_over_take");

  auto cbg = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);

  rclcpp::Clock system_clock(RCL_STEADY_TIME);
  std::mutex last_mutex;
  auto last = system_clock.now();

  std::atomic_int timer_count {0};

  auto timer_callback = [&timer_count, &executor, &system_clock, &last_mutex, &last]() {
      rclcpp::Time now = system_clock.now();
      timer_count++;

      if (timer_count > 20) {
        executor.cancel();
      }

      {
        std::lock_guard<std::mutex> lock(last_mutex);
        double diff = labs((now - last).nanoseconds()) / 1.0e9;
        last = now;

        if (diff < 0.009 || diff > 0.011) {
          executor.cancel();
          ASSERT_TRUE(diff > 0.009);
          ASSERT_TRUE(diff < 0.011);
        }
      }
    };

  auto timer = node->create_wall_timer(10ms, timer_callback, cbg);
  executor.add_node(node);
  executor.spin();
}
