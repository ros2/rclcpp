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
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestExecutors : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();
    node = std::make_shared<rclcpp::Node>("node", test_name.str());

    callback_count = 0;

    const std::string topic_name = std::string("topic_") + test_name.str();
    publisher = node->create_publisher<test_msgs::msg::Empty>(topic_name, rclcpp::QoS(10));
    auto callback = [this](test_msgs::msg::Empty::ConstSharedPtr) {this->callback_count++;};
    subscription =
      node->create_subscription<test_msgs::msg::Empty>(
      topic_name, rclcpp::QoS(10), std::move(callback));
  }

  void TearDown()
  {
    publisher.reset();
    subscription.reset();
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher;
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  int callback_count;
};

// spin_all and spin_some are not implemented correctly in StaticSingleThreadedExecutor, see:
// https://github.com/ros2/rclcpp/issues/1219 for tracking
template<typename T>
class TestExecutorsStable : public TestExecutors<T> {};

using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor>;

class ExecutorTypeNames
{
public:
  template<typename T>
  static std::string GetName(int idx)
  {
    (void)idx;
    if (std::is_same<T, rclcpp::executors::SingleThreadedExecutor>()) {
      return "SingleThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::MultiThreadedExecutor>()) {
      return "MultiThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::StaticSingleThreadedExecutor>()) {
      return "StaticSingleThreadedExecutor";
    }

    return "";
  }
};

// TYPED_TEST_SUITE is deprecated as of gtest 1.9, use TYPED_TEST_SUITE when gtest dependency
// is updated.
TYPED_TEST_SUITE(TestExecutors, ExecutorTypes, ExecutorTypeNames);

// StaticSingleThreadedExecutor is not included in these tests for now, due to:
// https://github.com/ros2/rclcpp/issues/1219
using StandardExecutors =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor>;
TYPED_TEST_SUITE(TestExecutorsStable, StandardExecutors, ExecutorTypeNames);

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
// Currently fails for StaticSingleThreadedExecutor so it is being skipped, see:
// https://github.com/ros2/rclcpp/issues/1231
TYPED_TEST(TestExecutorsStable, addTemporaryNode) {
  using ExecutorType = TypeParam;
  ExecutorType executor;

  {
    // Let node go out of scope before executor.spin()
    auto node = std::make_shared<rclcpp::Node>("temporary_node");
    executor.add_node(node);
  }

  // Sleep for a short time to verify executor.spin() is going, and didn't throw.
  std::thread spinner([&]() {EXPECT_NO_THROW(executor.spin());});

  std::this_thread::sleep_for(50ms);
  executor.cancel();
  spinner.join();
}

// Check executor throws properly if the same node is added a second time
TYPED_TEST(TestExecutors, addNodeTwoExecutors) {
  using ExecutorType = TypeParam;
  ExecutorType executor1;
  ExecutorType executor2;
  EXPECT_NO_THROW(executor1.add_node(this->node));
  EXPECT_THROW(executor2.add_node(this->node), std::runtime_error);
  executor1.remove_node(this->node, true);
}

// Check simple spin example
TYPED_TEST(TestExecutors, spinWithTimer) {
  using ExecutorType = TypeParam;
  ExecutorType executor;

  bool timer_completed = false;
  auto timer = this->node->create_wall_timer(1ms, [&]() {timer_completed = true;});
  executor.add_node(this->node);

  std::thread spinner([&]() {executor.spin();});

  auto start = std::chrono::steady_clock::now();
  while (!timer_completed && (std::chrono::steady_clock::now() - start) < 10s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(timer_completed);
  // Cancel needs to be called before join, so that executor.spin() returns.
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

TYPED_TEST(TestExecutors, spinWhileAlreadySpinning) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  bool timer_completed = false;
  auto timer = this->node->create_wall_timer(1ms, [&]() {timer_completed = true;});

  std::thread spinner([&]() {executor.spin();});
  // Sleep for a short time to verify executor.spin() is going, and didn't throw.

  auto start = std::chrono::steady_clock::now();
  while (!timer_completed && (std::chrono::steady_clock::now() - start) < 10s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(timer_completed);
  EXPECT_THROW(executor.spin(), std::runtime_error);

  // Shutdown needs to be called before join, so that executor.spin() returns.
  executor.cancel();
  spinner.join();
  executor.remove_node(this->node, true);
}

// Check executor exits immediately if future is complete.
TYPED_TEST(TestExecutors, testSpinUntilFutureComplete) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);

  // spin_until_future_complete is expected to exit immediately, but would block up until its
  // timeout if the future is not checked before spin_once_impl.
  auto start = std::chrono::steady_clock::now();
  auto shared_future = future.share();
  auto ret = executor.spin_until_future_complete(shared_future, 1s);
  executor.remove_node(this->node, true);
  // Check it didn't reach timeout
  EXPECT_GT(500ms, (std::chrono::steady_clock::now() - start));
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
}

// Same test, but uses a shared future.
TYPED_TEST(TestExecutors, testSpinUntilSharedFutureComplete) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);

  // spin_until_future_complete is expected to exit immediately, but would block up until its
  // timeout if the future is not checked before spin_once_impl.
  auto shared_future = future.share();
  auto start = std::chrono::steady_clock::now();
  auto ret = executor.spin_until_future_complete(shared_future, 1s);
  executor.remove_node(this->node, true);

  // Check it didn't reach timeout
  EXPECT_GT(500ms, (std::chrono::steady_clock::now() - start));
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
}

// For a longer running future that should require several iterations of spin_once
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteNoTimeout) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  // This future doesn't immediately terminate, so some work gets performed.
  std::future<void> future = std::async(
    std::launch::async,
    [this]() {
      auto start = std::chrono::steady_clock::now();
      while (this->callback_count < 1 && (std::chrono::steady_clock::now() - start) < 1s) {
        std::this_thread::sleep_for(1ms);
      }
    });

  bool spin_exited = false;

  // Timeout set to negative for no timeout.
  std::thread spinner([&]() {
      auto ret = executor.spin_until_future_complete(future, -1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
      executor.remove_node(this->node, true);
      executor.cancel();
      spin_exited = true;
    });

  // Do some work for longer than the future needs.
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(test_msgs::msg::Empty());
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
  executor.cancel();
  spinner.join();
}

// Check spin_until_future_complete timeout works as expected
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteWithTimeout) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  bool spin_exited = false;

  // Needs to run longer than spin_until_future_complete's timeout.
  std::future<void> future = std::async(
    std::launch::async,
    [&spin_exited]() {
      auto start = std::chrono::steady_clock::now();
      while (!spin_exited && (std::chrono::steady_clock::now() - start) < 1s) {
        std::this_thread::sleep_for(1ms);
      }
    });

  // Short timeout
  std::thread spinner([&]() {
      auto ret = executor.spin_until_future_complete(future, 1ms);
      EXPECT_EQ(rclcpp::FutureReturnCode::TIMEOUT, ret);
      executor.remove_node(this->node, true);
      spin_exited = true;
    });

  // Do some work for longer than timeout needs.
  for (int i = 0; i < 100; ++i) {
    this->publisher->publish(test_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
    if (spin_exited) {
      break;
    }
  }

  EXPECT_TRUE(spin_exited);
  spinner.join();
}

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable() = default;

  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    rclcpp::detail::add_guard_condition_to_rcl_wait_set(*wait_set, gc_);
  }

  void trigger()
  {
    gc_.trigger();
  }

  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return true;
  }

  std::shared_ptr<void>
  take_data() override
  {
    return nullptr;
  }

  void
  execute(std::shared_ptr<void> & data) override
  {
    (void) data;
    count_++;
    std::this_thread::sleep_for(3ms);
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
  rclcpp::GuardCondition gc_;
};

TYPED_TEST(TestExecutors, spinAll) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  auto waitable_interfaces = this->node->get_node_waitables_interface();
  auto my_waitable = std::make_shared<TestWaitable>();
  waitable_interfaces->add_waitable(my_waitable, nullptr);
  executor.add_node(this->node);

  // Long timeout, but should not block test if spin_all works as expected as we cancel the
  // executor.
  bool spin_exited = false;
  std::thread spinner([&spin_exited, &executor, this]() {
      executor.spin_all(1s);
      executor.remove_node(this->node, true);
      spin_exited = true;
    });

  // Do some work until sufficient calls to the waitable occur
  auto start = std::chrono::steady_clock::now();
  while (
    my_waitable->get_count() <= 1 &&
    !spin_exited &&
    (std::chrono::steady_clock::now() - start < 1s))
  {
    my_waitable->trigger();
    this->publisher->publish(test_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
  }

  executor.cancel();
  start = std::chrono::steady_clock::now();
  while (!spin_exited && (std::chrono::steady_clock::now() - start) < 1s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_LT(1u, my_waitable->get_count());
  waitable_interfaces->remove_waitable(my_waitable, nullptr);
  ASSERT_TRUE(spin_exited);
  spinner.join();
}

TYPED_TEST(TestExecutors, spinSome) {
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
      executor.remove_node(this->node, true);
      spin_exited = true;
    });

  // Do some work until sufficient calls to the waitable occur, but keep going until either
  // count becomes too large, spin exits, or the 1 second timeout completes.
  auto start = std::chrono::steady_clock::now();
  while (
    my_waitable->get_count() <= 1 &&
    !spin_exited &&
    (std::chrono::steady_clock::now() - start < 1s))
  {
    my_waitable->trigger();
    this->publisher->publish(test_msgs::msg::Empty());
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(1u, my_waitable->get_count());
  waitable_interfaces->remove_waitable(my_waitable, nullptr);
  EXPECT_TRUE(spin_exited);
  // Cancel if it hasn't exited already.
  executor.cancel();

  spinner.join();
}

// Check spin_node_until_future_complete with node base pointer
TYPED_TEST(TestExecutors, testSpinNodeUntilFutureCompleteNodeBasePtr) {
  using ExecutorType = TypeParam;
  ExecutorType executor;

  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);

  auto shared_future = future.share();
  auto ret = rclcpp::executors::spin_node_until_future_complete(
    executor, this->node->get_node_base_interface(), shared_future, 1s);
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
}

// Check spin_node_until_future_complete with node pointer
TYPED_TEST(TestExecutors, testSpinNodeUntilFutureCompleteNodePtr) {
  using ExecutorType = TypeParam;
  ExecutorType executor;

  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);

  auto shared_future = future.share();
  auto ret = rclcpp::executors::spin_node_until_future_complete(
    executor, this->node, shared_future, 1s);
  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
}

// Check spin_until_future_complete can be properly interrupted.
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteInterrupted) {
  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  bool spin_exited = false;

  // This needs to block longer than it takes to get to the shutdown call below and for
  // spin_until_future_complete to return
  std::future<void> future = std::async(
    std::launch::async,
    [&spin_exited]() {
      auto start = std::chrono::steady_clock::now();
      while (!spin_exited && (std::chrono::steady_clock::now() - start) < 1s) {
        std::this_thread::sleep_for(1ms);
      }
    });

  // Long timeout
  std::thread spinner([&spin_exited, &executor, &future]() {
      auto ret = executor.spin_until_future_complete(future, 1s);
      EXPECT_EQ(rclcpp::FutureReturnCode::INTERRUPTED, ret);
      spin_exited = true;
    });

  // Do some minimal work
  this->publisher->publish(test_msgs::msg::Empty());
  std::this_thread::sleep_for(1ms);

  // Force interruption
  rclcpp::shutdown();

  // Give it time to exit
  auto start = std::chrono::steady_clock::now();
  while (!spin_exited && (std::chrono::steady_clock::now() - start) < 1s) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(spin_exited);
  spinner.join();
}

// Check spin_until_future_complete with node base pointer (instantiates its own executor)
TEST(TestExecutors, testSpinUntilFutureCompleteNodeBasePtr) {
  rclcpp::init(0, nullptr);

  {
    auto node = std::make_shared<rclcpp::Node>("node");

    std::promise<bool> promise;
    std::future<bool> future = promise.get_future();
    promise.set_value(true);

    auto shared_future = future.share();
    auto ret = rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), shared_future, 1s);
    EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  }

  rclcpp::shutdown();
}

// Check spin_until_future_complete with node pointer (instantiates its own executor)
TEST(TestExecutors, testSpinUntilFutureCompleteNodePtr) {
  rclcpp::init(0, nullptr);

  {
    auto node = std::make_shared<rclcpp::Node>("node");

    std::promise<bool> promise;
    std::future<bool> future = promise.get_future();
    promise.set_value(true);

    auto shared_future = future.share();
    auto ret = rclcpp::spin_until_future_complete(node, shared_future, 1s);
    EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  }

  rclcpp::shutdown();
}
