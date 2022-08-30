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
#include <utility>

#include "rcl/timer.h"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

using namespace std::chrono_literals;

/// Timer testing bring up and teardown
class TestTimer : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    has_timer_run.store(false);
    cancel_timer.store(false);

    test_node = std::make_shared<rclcpp::Node>("test_timer_node");

    timer = test_node->create_wall_timer(
      100ms,
      [this]() -> void
      {
        this->has_timer_run.store(true);

        if (this->cancel_timer.load()) {
          this->timer->cancel();
        }
        // prevent any tests running timer from blocking
        this->executor->cancel();
      }
    );
    EXPECT_TRUE(timer->is_steady());

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
  std::atomic<bool> has_timer_run;
  // flag used to cancel the timer in the timer callback. If true cancel the timer, otherwise
  // cancel the executor (preventing any tests from blocking)
  std::atomic<bool> cancel_timer;
  rclcpp::Node::SharedPtr test_node;
  std::shared_ptr<rclcpp::TimerBase> timer;
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
TEST_F(TestTimer, test_simple_cancel)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);

  // cancel
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

  EXPECT_FALSE(has_timer_run.load());
}

/// Test state when using reset
TEST_F(TestTimer, test_is_canceled_reset)
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
TEST_F(TestTimer, test_run_cancel_executor)
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
TEST_F(TestTimer, test_run_cancel_timer)
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

TEST_F(TestTimer, test_bad_arguments) {
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
}

TEST_F(TestTimer, callback_with_timer) {
  rclcpp::TimerBase * timer_ptr = nullptr;
  timer = test_node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&timer_ptr](rclcpp::TimerBase & timer) {
      timer_ptr = &timer;
    });
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

TEST_F(TestTimer, callback_with_period_zero) {
  rclcpp::TimerBase * timer_ptr = nullptr;
  timer = test_node->create_wall_timer(
    std::chrono::milliseconds(0),
    [&timer_ptr](rclcpp::TimerBase & timer) {
      timer_ptr = &timer;
    });
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
TEST_F(TestTimer, test_failures_with_exceptions)
{
  // expect clean state, don't run otherwise
  test_initial_conditions(timer, has_timer_run);
  {
    std::shared_ptr<rclcpp::TimerBase> timer_to_test_destructor;
    // Test destructor failure, just logs a msg
    auto mock = mocking_utils::inject_on_return("lib:rclcpp", rcl_timer_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(
    {
      timer_to_test_destructor =
      test_node->create_wall_timer(std::chrono::milliseconds(0), [](void) {});
      timer_to_test_destructor.reset();
    });
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
