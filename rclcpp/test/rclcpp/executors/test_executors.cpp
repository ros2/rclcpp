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

/**
 * This test checks all implementations of rclcpp::executor to check they pass they basic API
 * tests. Anything specific to any executor in particular should go in a separate test file.
 *
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestExecutors : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("node", "ns");

    callback_count = 0;
    publisher = node->create_publisher<std_msgs::msg::Empty>("topic", rclcpp::QoS(10));
    auto callback = [this](std_msgs::msg::Empty::SharedPtr) {this->callback_count++;};
    subscription =
      node->create_subscription<std_msgs::msg::Empty>(
      "topic", rclcpp::QoS(10), std::move(callback));
  }

  void TearDown()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription;
  int callback_count;
};

// spin_all and spin_some are not implemented correctly in StaticSingleThreadedExecutor, see:
// https://github.com/ros2/rclcpp/issues/1219 for tracking
template<typename T>
class TestExecutorsSpinVariants : public TestExecutors<T> {};

using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor>;

TYPED_TEST_CASE(TestExecutors, ExecutorTypes);

// StaticSingleThreadedExecutor is not included in these tests for now, due to:
// https://github.com/ros2/rclcpp/issues/1219
using StandardExecutors =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor>;
TYPED_TEST_CASE(TestExecutorsSpinVariants, StandardExecutors);

// Make sure that executors detach from nodes when destructing
TYPED_TEST(TestExecutors, detachOnDestruction) {
  using ExecutorType = TypeParam;
  {
    ExecutorType executor;
    executor.add_node(this->node);
  }
  {
    ExecutorType executor;
    EXPECT_NO_THROW(executor.add_node(this->node));
  }
}

// Make sure that the executor can automatically remove expired nodes correctly
TYPED_TEST(TestExecutorsSpinVariants, addTemporaryNode) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(std::make_shared<rclcpp::Node>("temporary_node"));

  // Sleep for a short time to verify executor.spin() is going, and didn't throw.
  std::thread spinner([&]() {EXPECT_NO_THROW(executor.spin());});

  std::this_thread::sleep_for(50ms);
  rclcpp::shutdown();
  spinner.join();
}

// Make sure that the executor can automatically remove expired nodes correctly
TYPED_TEST(TestExecutors, addNodeTwoExecutors) {
  using ExecutorType = TypeParam;
  ExecutorType executor1;
  ExecutorType executor2;
  EXPECT_NO_THROW(executor1.add_node(this->node));
  EXPECT_THROW(executor2.add_node(this->node), std::runtime_error);
}

// Check simple spin example
TYPED_TEST(TestExecutors, spinWithTimer) {
  using ExecutorType = TypeParam;
  ExecutorType executor;

  bool timer_completed = false;
  auto timer = this->node->create_wall_timer(1ms, [&]() {timer_completed = true;});
  executor.add_node(this->node);

  std::thread spinner([&]() {executor.spin();});
  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(timer_completed);
  rclcpp::shutdown();
  spinner.join();
}

TYPED_TEST(TestExecutors, spinWhileAlreadySpinning) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  std::thread spinner([&]() {executor.spin();});
  // Sleep for a short time to verify executor.spin() is going, and didn't throw.
  std::this_thread::sleep_for(50ms);

  EXPECT_THROW(executor.spin(), std::runtime_error);
  rclcpp::shutdown();
  spinner.join();
}

// Check executor exits immediately if future is complete.
TYPED_TEST(TestExecutors, testSpinUntilFutureComplete) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);
  std::future<void> future;

  // test success of an immediately finishing future
  future = std::async(
    std::launch::async,
    []() {
      return;
    });

  bool spin_exited = false;

  std::thread spinner([&]() {
      auto ret = executor.spin_until_future_complete(future, 1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
      spin_exited = true;
    });

  // Do some work for longer than the future needs.
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  EXPECT_TRUE(spin_exited);
  spinner.join();
}

// Same test, but uses a shared future.
TYPED_TEST(TestExecutors, testSpinUntilSharedFutureComplete) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);
  std::future<void> future;

  // test success of an immediately finishing future
  future = std::async(
    std::launch::async,
    []() {
      return;
    });

  bool spin_exited = false;

  std::thread spinner([&]() {
      auto shared_future = future.share();
      auto ret = executor.spin_until_future_complete(shared_future, 1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
      spin_exited = true;
    });

  // Do some work for longer than the future needs.
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  EXPECT_TRUE(spin_exited);
  spinner.join();
}


// For a longer running future that should require several iterations of spin_once
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteNoTimeout) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // This future doesn't immediately terminate, so some work gets performed.
  std::future<void> future = std::async(
    std::launch::async,
    []() {std::this_thread::sleep_for(10ms);});

  bool spin_exited = false;

  // Timeout set to negative for no timeout.
  std::thread spinner([&]() {
      auto ret = executor.spin_until_future_complete(future, -1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
      spin_exited = true;
    });

  // Do some work for longer than the future needs.
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  // Not testing accuracy, just want to make sure that some work occurred.
  EXPECT_LT(0, this->callback_count);

  // If this fails, the test will probably crash because spinner goes out of scope while the thread
  // is active. However, it beats letting this run until the gtest timeout.
  ASSERT_TRUE(spin_exited);
  spinner.join();
}

// Check spin_until_future_complete timeout works as expected
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteWithTimeout) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // Long running future relative to timeout. This timeout blocks the test finishing, so it
  // shouldn't be too long.
  std::future<void> future = std::async(
    std::launch::async,
    []() {std::this_thread::sleep_for(20ms);});

  bool spin_exited = false;

  // Short timeout
  std::thread spinner([&]() {
      auto ret = executor.spin_until_future_complete(future, 10ms);
      EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
      spin_exited = true;
    });

  // Do some work for longer than timeout needs.
  for (int i = 0; i < 10; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  // Not testing accuracy, just want to make sure that some work occurred.
  EXPECT_LT(0, this->callback_count);

  EXPECT_TRUE(spin_exited);
  spinner.join();
}

// Check spin_until_future_complete can be properly interrupted.
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteInterrupted) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // This needs to be longer than it takes to reach the shutdown call below.
  // This timeout blocks the test finishing, so it shouldn't be too long.
  std::future<void> future = std::async(
    std::launch::async,
    []() {std::this_thread::sleep_for(20ms);});

  bool spin_exited = false;

  // Long timeout
  std::thread spinner([&spin_exited, &executor, &future]() {
      auto ret = executor.spin_until_future_complete(future, 1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::INTERRUPTED, ret);
      spin_exited = true;
    });

  // Do some minimal work
  this->publisher->publish(std_msgs::msg::Empty());
  std::this_thread::sleep_for(1ms);

  // Force interruption
  rclcpp::shutdown();

  // Give it time to exit
  std::this_thread::sleep_for(10ms);
  EXPECT_TRUE(spin_exited);
  spinner.join();
}

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable()
  {
    rcl_guard_condition_options_t guard_condition_options =
      rcl_guard_condition_get_default_options();

    gc_ = rcl_get_zero_initialized_guard_condition();
    rcl_ret_t ret = rcl_guard_condition_init(
      &gc_,
      rclcpp::contexts::get_global_default_context()->get_rcl_context().get(),
      guard_condition_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &gc_, NULL);
    if (RCL_RET_OK != ret) {
      return false;
    }
    ret = rcl_trigger_guard_condition(&gc_);
    return RCL_RET_OK == ret;
  }

  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return true;
  }

  void
  execute() override
  {
    count_++;
    std::this_thread::sleep_for(1ms);
  }

  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  size_t
  get_count()
  {
    return count_;
  }

private:
  size_t count_ = 0;
  rcl_guard_condition_t gc_;
};

TYPED_TEST(TestExecutorsSpinVariants, spinAll) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  auto waitable_interfaces = this->node->get_node_waitables_interface();
  auto my_waitable = std::make_shared<TestWaitable>();
  waitable_interfaces->add_waitable(my_waitable, nullptr);
  executor.add_node(this->node);

  // Long timeout, this determines the duration of the test, so not making it too long.
  // Just long enough for multiple waitables to execute.
  bool spin_exited = false;
  std::thread spinner([&spin_exited, &executor, this]() {
      executor.spin_all(20ms);
      executor.remove_node(this->node);
      spin_exited = true;
    });

  // Do some work for longer the test waitable needs
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  EXPECT_GT(my_waitable->get_count(), 1u);
  waitable_interfaces->remove_waitable(my_waitable, nullptr);
  ASSERT_TRUE(spin_exited);
  spinner.join();
}

TYPED_TEST(TestExecutorsSpinVariants, spinSome) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  auto waitable_interfaces = this->node->get_node_waitables_interface();
  auto my_waitable = std::make_shared<TestWaitable>();
  waitable_interfaces->add_waitable(my_waitable, nullptr);
  executor.add_node(this->node);

  // Long timeout, doesn't block test from finishing because spin_some should exit after the
  // first one completes.
  bool spin_exited = false;
  std::thread spinner([&spin_exited, &executor, this]() {
      executor.spin_some(1s);
      executor.remove_node(this->node);
      spin_exited = true;
    });

  // Do some work for longer the test waitable needs
  for (int i = 0; i < 10; ++i) {
    this->publisher->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  EXPECT_EQ(my_waitable->get_count(), 1u);
  waitable_interfaces->remove_waitable(my_waitable, nullptr);
  ASSERT_TRUE(spin_exited);
  spinner.join();
}
