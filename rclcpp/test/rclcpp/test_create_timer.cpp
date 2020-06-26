// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "node_interfaces/node_wrapper.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

using namespace std::chrono_literals;

TEST(TestCreateTimer, timer_executes)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_create_timer_node");

  std::atomic<bool> got_callback{false};

  rclcpp::TimerBase::SharedPtr timer;
  timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(0ms),
    [&got_callback, &timer]() {
      got_callback = true;
      timer->cancel();
    });

  rclcpp::spin_some(node);

  ASSERT_TRUE(got_callback);
  rclcpp::shutdown();
}

TEST(TestCreateTimer, call_with_node_wrapper_compiles)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_timer_call_with_node_wrapper_compiles");

  rclcpp::TimerBase::SharedPtr timer;
  timer = rclcpp::create_timer(
    node,
    node.get_node_clock_interface()->get_clock(),
    rclcpp::Duration(0ms),
    []() {});
  rclcpp::shutdown();
}

TEST(TestCreateTimer, call_wall_timer_with_bad_arguments)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_wall_timers_with_bad_arguments");
  auto callback = []() {};
  rclcpp::CallbackGroup::SharedPtr group = nullptr;
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();

  // Negative period
  EXPECT_THROW(
    rclcpp::create_wall_timer(-1ms, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // Very negative period
  constexpr auto nanoseconds_min = std::chrono::nanoseconds::min();
  EXPECT_THROW(
    rclcpp::create_wall_timer(
      nanoseconds_min, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // Period must be less than nanoseconds::max()
  constexpr auto nanoseconds_max = std::chrono::nanoseconds::min();
  EXPECT_THROW(
    rclcpp::create_wall_timer(
      nanoseconds_max, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  EXPECT_NO_THROW(
    rclcpp::create_wall_timer(
      nanoseconds_max - 1us, callback, group, node_interface, timers_interface));

  EXPECT_NO_THROW(
    rclcpp::create_wall_timer(0ms, callback, group, node_interface, timers_interface));

  // Period must be less than nanoseconds::max()
  constexpr auto hours_max = std::chrono::hours::max();
  EXPECT_THROW(
    rclcpp::create_wall_timer(hours_max, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // node_interface is null
  EXPECT_THROW(
    rclcpp::create_wall_timer(1ms, callback, group, nullptr, timers_interface),
    std::invalid_argument);

  // timers_interface is null
  EXPECT_THROW(
    rclcpp::create_wall_timer(1ms, callback, group, node_interface, nullptr),
    std::invalid_argument);
  rclcpp::shutdown();
}
