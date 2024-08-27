// Copyright 2024 iRobot Corporation.
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
#include <cstddef>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "./executor_types.hpp"

template<typename ExecutorType>
class TestTimersLifecycle : public testing::Test
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}

  void TearDown() override {rclcpp::shutdown();}
};

TYPED_TEST_SUITE(TestTimersLifecycle, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestTimersLifecycle, timers_lifecycle_reinitialized_object)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto timers_period = std::chrono::milliseconds(50);
  auto node = std::make_shared<rclcpp::Node>("test_node");

  executor.add_node(node);

  size_t count_1 = 0;
  auto timer_1 = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(timers_period), [&count_1]() {count_1++;});

  size_t count_2 = 0;
  auto timer_2 = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(timers_period), [&count_2]() {count_2++;});

  {
    std::thread executor_thread([&executor]() {executor.spin();});

    while (count_2 < 10u) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.cancel();
    executor_thread.join();

    EXPECT_GE(count_2, 10u);
    EXPECT_LE(count_2 - count_1, 1u);
  }

  count_1 = 0;
  timer_1 = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(timers_period), [&count_1]() {count_1++;});

  count_2 = 0;
  timer_2 = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(timers_period), [&count_2]() {count_2++;});

  {
    std::thread executor_thread([&executor]() {executor.spin();});

    while (count_2 < 10u) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.cancel();
    executor_thread.join();

    EXPECT_GE(count_2, 10u);
    EXPECT_LE(count_2 - count_1, 1u);
  }
}
