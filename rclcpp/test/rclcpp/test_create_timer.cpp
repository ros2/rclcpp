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

#include "rclcpp/create_timer.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "node_interfaces/node_wrapper.hpp"

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
