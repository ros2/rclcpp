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

#include "rcl/timer.h"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

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

    timer = test_node->create_wall_timer(100ms,
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
  EXPECT_FALSE(timer->is_canceled());

  // cancel after reset
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

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
