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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono_literals;

class TestWakeAfterExecuteFlag : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};


constexpr int EXECUTION_COUNT = 5;

/*
   Test guard condition trigger is set when a node is added and resetted when it is removed
 */
TEST_F(TestWakeAfterExecuteFlag, set_wake_after_execute_flag_multi_threaded) {

  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_wake_after_execute_flag_multi_threaded");

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  std::atomic_int timer_count {0};

  auto timer_callback = [&executor, &timer_count]() {
    printf("Timer executed!");

    if(timer_count > 0){
      ASSERT_EQ(executor.get_wake_after_execute_flag(), true);
    }

    timer_count++;

    if(timer_count > EXECUTION_COUNT){
      executor.cancel();
    }
  };

  auto timer_ = node->create_wall_timer(
      2s, timer_callback, cbg);

  executor.add_node(node);
  executor.spin();
}

TEST_F(TestWakeAfterExecuteFlag, set_wake_after_execute_flag_single_threaded) {

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_wake_after_execute_flag_single_threaded");

  std::atomic_int timer_count {0};

  auto timer_callback = [&executor, &timer_count]() {
    printf("Timer executed!");

    if(timer_count > 0){
      ASSERT_EQ(executor.get_wake_after_execute_flag(), false);
    }

    timer_count++;

    if(timer_count > EXECUTION_COUNT){
      executor.cancel();
    }
  };

  auto timer_ = node->create_wall_timer(
      2s, timer_callback);

  executor.add_node(node);
  executor.spin();
}