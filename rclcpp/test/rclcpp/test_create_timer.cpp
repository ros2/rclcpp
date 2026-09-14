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

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();

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

TEST(TestCreateWallTimer, call_wall_timer_with_bad_arguments)
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

TEST(TestCreateTimer, call_timer_with_bad_arguments)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_timers_with_bad_arguments");
  auto callback = []() {};
  rclcpp::CallbackGroup::SharedPtr group = nullptr;
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();

  auto clock = node.get_node_clock_interface()->get_clock();

  // Negative period
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, -1ms, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // Very negative period
  constexpr auto nanoseconds_min = std::chrono::nanoseconds::min();
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, nanoseconds_min, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // Period must be less than nanoseconds::max()
  constexpr auto nanoseconds_max = std::chrono::nanoseconds::min();
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, nanoseconds_max, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  EXPECT_NO_THROW(
    rclcpp::create_timer(
      clock, nanoseconds_max - 1us, callback, group, node_interface, timers_interface));

  EXPECT_NO_THROW(
    rclcpp::create_timer(clock, 0ms, callback, group, node_interface, timers_interface));

  // Period must be less than nanoseconds::max()
  constexpr auto hours_max = std::chrono::hours::max();
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, hours_max, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // node_interface is null
  EXPECT_THROW(
    rclcpp::create_timer(clock, 1ms, callback, group, nullptr, timers_interface),
    std::invalid_argument);

  // timers_interface is null
  EXPECT_THROW(
    rclcpp::create_timer(clock, 1ms, callback, group, node_interface, nullptr),
    std::invalid_argument);

  rclcpp::shutdown();
}

static void test_timer_callback(void) {}

TEST(TestCreateTimer, timer_function_pointer)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("timer_function_pointer_node");

  // make sure build succeeds with function pointer instead of lambda
  auto some_timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(0ms),
    test_timer_callback);

  rclcpp::shutdown();
}

TEST(TestCreateTimer, call_timer_with_initial_call_time_bad_arguments)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_timers_with_initial_call_time_bad_arguments");
  auto callback = []() {};
  rclcpp::CallbackGroup::SharedPtr group = nullptr;
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();

  auto clock = node.get_node_clock_interface()->get_clock();
  auto initial_call_time = clock->now() + rclcpp::Duration(1s);

  // Negative period
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, initial_call_time, -1ms, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // clock is null
  EXPECT_THROW(
    rclcpp::create_timer(
      nullptr, initial_call_time, 1ms, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // node_interface is null
  EXPECT_THROW(
    rclcpp::create_timer(clock, initial_call_time, 1ms, callback, group, nullptr, timers_interface),
    std::invalid_argument);

  // timers_interface is null
  EXPECT_THROW(
    rclcpp::create_timer(clock, initial_call_time, 1ms, callback, group, node_interface, nullptr),
    std::invalid_argument);

  // initial_call_time's clock type does not match clock's clock type
  rclcpp::Time mismatched_clock_type_time(static_cast<int64_t>(0), RCL_STEADY_TIME);
  EXPECT_THROW(
    rclcpp::create_timer(
      clock, mismatched_clock_type_time, 1ms, callback, group, node_interface, timers_interface),
    std::runtime_error);

  // Same check for create_wall_timer, which always uses a steady clock internally regardless of
  // what clock type initial_call_time was constructed with.
  EXPECT_THROW(
    rclcpp::create_wall_timer(
      initial_call_time, 1ms, callback, group, node_interface, timers_interface),
    std::runtime_error);

  rclcpp::shutdown();
}

TEST(TestCreateTimer, call_timer_honors_initial_call_time)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_timer_honors_initial_call_time");
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();
  auto clock = node.get_node_clock_interface()->get_clock();

  const auto period = 50ms;
  const auto initial_delay = 10s;
  auto initial_call_time = clock->now() + rclcpp::Duration(initial_delay);

  auto timer = rclcpp::create_timer(
    clock, initial_call_time, period, []() {}, nullptr, node_interface, timers_interface);

  // The next call should be driven by initial_call_time, not by now + period.
  EXPECT_GT(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(period).count());
  EXPECT_LE(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(initial_delay).count());

  timer->cancel();
  rclcpp::shutdown();
}

TEST(TestCreateTimer, call_timer_forwards_autostart_regardless_of_initial_call_time_overload)
{
  // Regression test: rclcpp::create_timer(clock, period, callback, group, node_base, node_timers,
  // autostart) must honor a false autostart, rather than always starting the timer, when it
  // delegates to the initial-call-time overload internally.
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_timer_forwards_autostart");
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();
  auto clock = node.get_node_clock_interface()->get_clock();

  auto timer = rclcpp::create_timer(
    clock, 100ms, []() {}, nullptr, node_interface, timers_interface, false);

  EXPECT_TRUE(timer->is_canceled());
  timer->reset();
  EXPECT_FALSE(timer->is_canceled());
  timer->cancel();

  rclcpp::shutdown();
}

TEST(TestCreateWallTimer, call_wall_timer_with_initial_call_time_bad_arguments)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_wall_timer_with_initial_call_time_bad_arguments");
  auto callback = []() {};
  rclcpp::CallbackGroup::SharedPtr group = nullptr;
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();

  auto initial_call_time = rclcpp::Clock(RCL_STEADY_TIME).now() + rclcpp::Duration(1s);

  // Negative period
  EXPECT_THROW(
    rclcpp::create_wall_timer(
      initial_call_time, -1ms, callback, group, node_interface, timers_interface),
    std::invalid_argument);

  // node_interface is null
  EXPECT_THROW(
    rclcpp::create_wall_timer(initial_call_time, 1ms, callback, group, nullptr, timers_interface),
    std::invalid_argument);

  // timers_interface is null
  EXPECT_THROW(
    rclcpp::create_wall_timer(initial_call_time, 1ms, callback, group, node_interface, nullptr),
    std::invalid_argument);

  rclcpp::shutdown();
}

TEST(TestCreateWallTimer, call_wall_timer_honors_initial_call_time)
{
  rclcpp::init(0, nullptr);
  NodeWrapper node("test_create_wall_timer_honors_initial_call_time");
  auto node_interface =
    rclcpp::node_interfaces::get_node_base_interface(node).get();
  auto timers_interface =
    rclcpp::node_interfaces::get_node_timers_interface(node).get();

  const auto period = 50ms;
  const auto initial_delay = 10s;
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  auto initial_call_time = steady_clock.now() + rclcpp::Duration(initial_delay);

  auto timer = rclcpp::create_wall_timer(
    initial_call_time, period, []() {}, nullptr, node_interface, timers_interface);

  // The next call should be driven by initial_call_time, not by now + period.
  EXPECT_GT(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(period).count());
  EXPECT_LE(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(initial_delay).count());

  timer->cancel();
  rclcpp::shutdown();
}

TEST(TestCreateTimer, timer_without_autostart)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_create_timer_node");

  rclcpp::TimerBase::SharedPtr timer;
  timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(0ms),
    []() {},
    nullptr,
    false);

  EXPECT_TRUE(timer->is_canceled());
  EXPECT_EQ(timer->time_until_trigger().count(), std::chrono::nanoseconds::max().count());

  timer->reset();
  EXPECT_LE(timer->time_until_trigger().count(), std::chrono::nanoseconds::max().count());
  EXPECT_FALSE(timer->is_canceled());

  timer->cancel();

  rclcpp::shutdown();
}
