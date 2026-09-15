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
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl/time.h"
#include "rcl/timer.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

using namespace std::chrono_literals;

/// We want to test everything for both the wall and generic timer.
enum class TimerType
{
  WALL_TIMER,
  GENERIC_TIMER,
};

/// Timer testing bring up and teardown
class TestTimer : public ::testing::TestWithParam<TimerType>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    has_timer_run.store(false);
    cancel_timer.store(false);

    test_node = std::make_shared<rclcpp::Node>("test_timer_node");

    auto timer_callback = [this]() -> void {
        this->has_timer_run.store(true);

        if (this->cancel_timer.load()) {
          this->timer->cancel();
        }
        // prevent any tests running timer from blocking
        this->executor->cancel();
      };

    // Store the timer type for use in TEST_P declarations.
    timer_type = GetParam();
    switch (timer_type) {
      case TimerType::WALL_TIMER:
        timer = test_node->create_wall_timer(100ms, timer_callback);
        EXPECT_TRUE(timer->is_steady());
        break;
      case TimerType::GENERIC_TIMER:
        timer = test_node->create_timer(100ms, timer_callback);
        EXPECT_FALSE(timer->is_steady());
        break;
    }
    timer_without_autostart = test_node->create_wall_timer(
      100ms,
      [this]() -> void
      {
        this->has_timer_run.store(true);

        if (this->cancel_timer.load()) {
          this->timer->cancel();
        }
        // prevent any tests running timer from blocking
        this->executor->cancel();
      }, nullptr, false);
    EXPECT_TRUE(timer_without_autostart->is_steady());

    executor->add_node(test_node);
    // don't start spinning, let the test dictate when
  }

  void TearDown() override
  {
    timer.reset();
    test_node.reset();
    executor.reset();
    rclcpp::shutdown();
  }

  // set to true if the timer callback executed, false otherwise
  TimerType timer_type;
  std::atomic<bool> has_timer_run;
  // flag used to cancel the timer in the timer callback. If true cancel the timer, otherwise
  // cancel the executor (preventing any tests from blocking)
  std::atomic<bool> cancel_timer;
  rclcpp::Node::SharedPtr test_node;
  std::shared_ptr<rclcpp::TimerBase> timer;
  std::shared_ptr<rclcpp::TimerBase> timer_without_autostart;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

/// check if initial states are set as expected
void test_initial_conditions(
  std::shared_ptr<rclcpp::TimerBase> & timer,
  std::atomic<bool> & has_timer_run)
{
  ASSERT_FALSE(timer->is_canceled());
  ASSERT_FALSE(has_timer_run.load());
}

/// Simple test
TEST_P(TestTimer, test_simple_cancel)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);

  // cancel
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

  EXPECT_FALSE(has_timer_run.load());
}

/// Test state when using reset
TEST_P(TestTimer, test_is_canceled_reset)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);

  // reset shouldn't affect state (not canceled yet)
  timer->reset();
  EXPECT_LE(timer->time_until_trigger().count(), std::chrono::nanoseconds::max().count());
  EXPECT_FALSE(timer->is_canceled());

  // cancel after reset
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());
  EXPECT_EQ(timer->time_until_trigger().count(), std::chrono::nanoseconds::max().count());

  // reset and cancel
  timer->reset();
  EXPECT_FALSE(timer->is_canceled());
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

  EXPECT_FALSE(has_timer_run.load());
}

/// Run and check state, cancel the executor
TEST_P(TestTimer, test_run_cancel_executor)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);

  // run the timer (once, this forces an executor cancel so spin won't block)
  // but the timer was not explicitly cancelled
  executor->spin();
  EXPECT_TRUE(has_timer_run.load());

  // force a timer cancel
  EXPECT_FALSE(timer->is_canceled());
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());
}

/// Run and check state, cancel the timer
TEST_P(TestTimer, test_run_cancel_timer)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);

  // force a timer cancellation
  cancel_timer.store(true);
  // run the timer (once, this forces an executor cancel so spin won't block)
  executor->spin();
  EXPECT_TRUE(has_timer_run.load());
  EXPECT_TRUE(timer->is_canceled());
}

TEST_P(TestTimer, test_bad_arguments) {
  auto node_base = rclcpp::node_interfaces::get_node_base_interface(test_node);
  auto context = node_base->get_context();

  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  // Negative period
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(steady_clock, -1ms, []() {}, context),
    rclcpp::exceptions::RCLInvalidArgument);

  // Very negative period
  constexpr auto nanoseconds_min = std::chrono::nanoseconds::min();
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(
      steady_clock, nanoseconds_min, []() {}, context),
    rclcpp::exceptions::RCLInvalidArgument);

  // nanoseconds max, should be ok
  constexpr auto nanoseconds_max = std::chrono::nanoseconds::max();
  EXPECT_NO_THROW(
    rclcpp::GenericTimer<void (*)()>(
      steady_clock, nanoseconds_max, []() {}, context));

  // 0 duration period, should be ok
  EXPECT_NO_THROW(
    rclcpp::GenericTimer<void (*)()>(steady_clock, 0ms, []() {}, context));

  // context is null, which resorts to default
  EXPECT_NO_THROW(
    rclcpp::GenericTimer<void (*)()>(steady_clock, 1ms, []() {}, nullptr));

  // Clock is unitialized
  auto unitialized_clock = std::make_shared<rclcpp::Clock>(RCL_CLOCK_UNINITIALIZED);
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(unitialized_clock, 1us, []() {}, context),
    rclcpp::exceptions::RCLError);

  // Same set of checks, but using the constructor that takes an explicit initial call time.
  auto now = steady_clock->now();

  // Negative period
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(steady_clock, now, -1ms, []() {}, context),
    rclcpp::exceptions::RCLInvalidArgument);

  // 0 duration period, should be ok
  EXPECT_NO_THROW(
    rclcpp::GenericTimer<void (*)()>(steady_clock, now, 0ms, []() {}, context));

  // Clock is unitialized
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(unitialized_clock, now, 1us, []() {}, context),
    rclcpp::exceptions::RCLError);

  // initial_call_time's clock type does not match the timer's clock
  rclcpp::Time mismatched_clock_type_time(static_cast<int64_t>(0), RCL_SYSTEM_TIME);
  EXPECT_THROW(
    rclcpp::GenericTimer<void (*)()>(
      steady_clock, mismatched_clock_type_time, 1ms, []() {}, context),
    std::runtime_error);

  // Same check for WallTimer, which always uses a steady clock internally regardless of what
  // clock type initial_call_time was constructed with.
  EXPECT_THROW(
    rclcpp::WallTimer<void (*)()>(mismatched_clock_type_time, 1ms, []() {}, context),
    std::runtime_error);
}

TEST_P(TestTimer, test_initial_call_time)
{
  const auto period = 50ms;
  const auto initial_delay = std::chrono::seconds(10);

  std::shared_ptr<rclcpp::TimerBase> initial_time_timer;
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        initial_time_timer = test_node->create_wall_timer(
          steady_clock.now() + rclcpp::Duration(initial_delay), period, []() {});
        break;
      }
    case TimerType::GENERIC_TIMER:
      {
        initial_time_timer = test_node->create_timer(
          test_node->get_clock()->now() + rclcpp::Duration(initial_delay), period, []() {});
        break;
      }
  }

  // The next call should be driven by initial_call_time, not by now + period.
  EXPECT_GT(
    initial_time_timer->time_until_trigger().count(),
    std::chrono::nanoseconds(period).count());
  EXPECT_LE(
    initial_time_timer->time_until_trigger().count(),
    std::chrono::nanoseconds(initial_delay).count());

  initial_time_timer->cancel();
}

TEST_P(TestTimer, resume_does_not_advance_if_not_yet_due)
{
  const auto period = 50ms;
  const auto initial_delay = std::chrono::seconds(10);

  std::shared_ptr<rclcpp::TimerBase> timer;
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        timer = test_node->create_wall_timer(
          steady_clock.now() + rclcpp::Duration(initial_delay), period, []() {},
          nullptr, false);
        break;
      }
    case TimerType::GENERIC_TIMER:
      {
        timer = test_node->create_timer(
          test_node->get_clock()->now() + rclcpp::Duration(initial_delay), period, []() {},
          nullptr, false);
        break;
      }
  }

  EXPECT_TRUE(timer->is_canceled());
  timer->resume();
  EXPECT_FALSE(timer->is_canceled());

  // The original phase-anchored schedule should be preserved, not recomputed from now().
  EXPECT_GT(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(period).count());
  EXPECT_LE(
    timer->time_until_trigger().count(),
    std::chrono::nanoseconds(initial_delay).count());

  timer->cancel();
}

TEST_P(TestTimer, resume_catches_up_if_overdue)
{
  // Simulate a timer that was paused (canceled) for much longer than several periods.
  const auto period = 100ms;
  const auto overdue_by = 1050ms;

  std::shared_ptr<rclcpp::TimerBase> timer;
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        timer = test_node->create_wall_timer(
          steady_clock.now() - rclcpp::Duration(overdue_by), period, []() {},
          nullptr, false);
        break;
      }
    case TimerType::GENERIC_TIMER:
      {
        timer = test_node->create_timer(
          test_node->get_clock()->now() - rclcpp::Duration(overdue_by), period, []() {},
          nullptr, false);
        break;
      }
  }

  timer->resume();
  EXPECT_FALSE(timer->is_canceled());

  // Should have caught up to the next period boundary after now, not restarted from now().
  EXPECT_GT(timer->time_until_trigger().count(), 0);
  EXPECT_LE(timer->time_until_trigger().count(), std::chrono::nanoseconds(period).count());

  timer->cancel();
}

TEST_P(TestTimer, resume_uncancels_a_canceled_timer)
{
  const auto period = 10s;

  std::shared_ptr<rclcpp::TimerBase> timer;
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      timer = test_node->create_wall_timer(period, []() {});
      break;
    case TimerType::GENERIC_TIMER:
      timer = test_node->create_timer(period, []() {});
      break;
  }

  const auto time_until_trigger_before = timer->time_until_trigger();

  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

  timer->resume();
  EXPECT_FALSE(timer->is_canceled());

  // A short cancel/resume cycle with a long period should not have shifted the phase.
  EXPECT_NEAR(
    static_cast<double>(time_until_trigger_before.count()),
    static_cast<double>(timer->time_until_trigger().count()),
    static_cast<double>(std::chrono::nanoseconds(500ms).count()));

  timer->cancel();
}

TEST_P(TestTimer, callback_with_timer) {
  rclcpp::TimerBase * timer_ptr = nullptr;
  auto timer_callback = [&timer_ptr](rclcpp::TimerBase & timer) {
      timer_ptr = &timer;
    };
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      timer = test_node->create_wall_timer(1ms, timer_callback);
      break;
    case TimerType::GENERIC_TIMER:
      timer = test_node->create_timer(1ms, timer_callback);
      break;
  }
  auto start = std::chrono::steady_clock::now();
  while (nullptr == timer_ptr &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(100))
  {
    executor->spin_once(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(timer.get(), timer_ptr);
  EXPECT_LE(std::chrono::nanoseconds(0).count(), timer_ptr->time_until_trigger().count());
  EXPECT_FALSE(timer_ptr->is_ready());
}

TEST_P(TestTimer, callback_with_timer_info) {
  rclcpp::TimerInfo info;
  auto timer_callback = [&info](const rclcpp::TimerInfo & timer_info) {
      info = timer_info;
    };
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      timer = test_node->create_wall_timer(1ms, timer_callback);
      break;
    case TimerType::GENERIC_TIMER:
      timer = test_node->create_timer(1ms, timer_callback);
      break;
  }
  auto start = std::chrono::steady_clock::now();
  while (info.actual_call_time.nanoseconds() == 0 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(100))
  {
    executor->spin_once(std::chrono::milliseconds(10));
  }
  EXPECT_GE(info.actual_call_time, info.expected_call_time);
}

TEST_P(TestTimer, callback_with_period_zero) {
  rclcpp::TimerBase * timer_ptr = nullptr;
  auto timer_callback = [&timer_ptr](rclcpp::TimerBase & timer) {
      timer_ptr = &timer;
    };
  switch (timer_type) {
    case TimerType::WALL_TIMER:
      timer = test_node->create_wall_timer(0ms, timer_callback);
      break;
    case TimerType::GENERIC_TIMER:
      timer = test_node->create_timer(0ms, timer_callback);
      break;
  }
  auto start = std::chrono::steady_clock::now();
  while (nullptr == timer_ptr &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(100))
  {
    executor->spin_once(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(timer.get(), timer_ptr);
  EXPECT_GE(std::chrono::nanoseconds(0).count(), timer_ptr->time_until_trigger().count());
  EXPECT_TRUE(timer_ptr->is_ready());
}

/// Test internal failures using mocks
TEST_P(TestTimer, test_failures_with_exceptions)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);
  {
    std::shared_ptr<rclcpp::TimerBase> timer_to_test_destructor;
    // Test destructor failure, just logs a msg
    auto mock = mocking_utils::inject_on_return("lib:rclcpp", rcl_timer_fini, RCL_RET_ERROR);
    if (timer_type == TimerType::WALL_TIMER) {
      timer_to_test_destructor =
        test_node->create_wall_timer(std::chrono::milliseconds(0), [](void) {});
    } else {
      timer_to_test_destructor =
        test_node->create_timer(std::chrono::milliseconds(0), [](void) {});
    }
    timer_to_test_destructor.reset();
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_timer_cancel, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      timer->cancel(), std::runtime_error("Couldn't cancel timer: error not set"));
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_timer_is_canceled, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      timer->is_canceled(),
      std::runtime_error("Couldn't get timer cancelled state: error not set"));
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_timer_reset, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      timer->reset(), std::runtime_error("Couldn't reset timer: error not set"));
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_timer_is_ready, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      timer->is_ready(), std::runtime_error("Failed to check timer: error not set"));
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_timer_get_time_until_next_call, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      timer->time_until_trigger(),
      std::runtime_error("Timer could not get time until next call: error not set"));
  }
}

INSTANTIATE_TEST_SUITE_P(
  PerTimerType, TestTimer,
  ::testing::Values(TimerType::WALL_TIMER, TimerType::GENERIC_TIMER),
  [](const ::testing::TestParamInfo<TimerType> & info) -> std::string {
    switch (info.param) {
      case TimerType::WALL_TIMER:
        return std::string("wall_timer");
      case TimerType::GENERIC_TIMER:
        return std::string("generic_timer");
      default:
        break;
    }
    return std::string("unknown");
  }
);

/// Simple test of a timer without autostart
TEST_P(TestTimer, test_timer_without_autostart)
{
  EXPECT_TRUE(timer_without_autostart->is_canceled());
  EXPECT_EQ(
    timer_without_autostart->time_until_trigger().count(),
    std::chrono::nanoseconds::max().count());
  // Reset to change start timer
  timer_without_autostart->reset();
  EXPECT_LE(
    timer_without_autostart->time_until_trigger().count(),
    std::chrono::nanoseconds::max().count());
  EXPECT_FALSE(timer_without_autostart->is_canceled());
}

class TestComputePhaseAlignedTime : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rcl_clock_ = clock_->get_clock_handle();
    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock_));
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void set_now(rcl_time_point_value_t now_ns)
  {
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock_, now_ns));
  }

  rclcpp::Clock::SharedPtr clock_;
  rcl_clock_t * rcl_clock_;
};

TEST_F(TestComputePhaseAlignedTime, rejects_non_positive_interval)
{
  set_now(1);
  EXPECT_THROW(
    rclcpp::compute_phase_aligned_time(*clock_, 0ns),
    std::invalid_argument);
  EXPECT_THROW(
    rclcpp::compute_phase_aligned_time(*clock_, -1ns),
    std::invalid_argument);
}

TEST_F(TestComputePhaseAlignedTime, advances_to_next_boundary_with_zero_phase)
{
  set_now(1'000'000'000);  // 1.0s
  auto result = rclcpp::compute_phase_aligned_time(*clock_, 300ms);
  EXPECT_EQ(1'200'000'000, result.nanoseconds());
}

TEST_F(TestComputePhaseAlignedTime, returns_now_when_already_on_a_boundary)
{
  set_now(900'000'000);  // 0.9s, an exact multiple of the 300ms interval
  auto result = rclcpp::compute_phase_aligned_time(*clock_, 300ms);
  EXPECT_EQ(900'000'000, result.nanoseconds());
}

TEST_F(TestComputePhaseAlignedTime, honors_phase_offset)
{
  set_now(1'000'000'000);  // 1.0s
  auto result = rclcpp::compute_phase_aligned_time(*clock_, 300ms, 50ms);
  EXPECT_EQ(1'250'000'000, result.nanoseconds());
}

TEST_F(TestComputePhaseAlignedTime, result_uses_clocks_type)
{
  set_now(1);
  auto result = rclcpp::compute_phase_aligned_time(*clock_, 300ms);
  EXPECT_EQ(RCL_ROS_TIME, result.get_clock_type());
}
