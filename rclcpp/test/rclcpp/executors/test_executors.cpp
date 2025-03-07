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
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "./executor_types.hpp"
#include "./test_waitable.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestExecutors : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);

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

    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher;
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  int callback_count;
};

template<typename T>
class TestExecutorsStable : public TestExecutors<T> {};

TYPED_TEST_SUITE(TestExecutors, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST_SUITE(TestExecutorsStable, StandardExecutors, ExecutorTypeNames);

// Make sure that executors detach from nodes when destructing
TYPED_TEST(TestExecutors, detachOnDestruction)
{
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
TYPED_TEST(TestExecutors, addTemporaryNode) {
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

// Make sure that a spinning empty executor can be cancelled
TYPED_TEST(TestExecutors, emptyExecutor)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;
  std::thread spinner([&]() {EXPECT_NO_THROW(executor.spin());});
  std::this_thread::sleep_for(50ms);
  executor.cancel();
  spinner.join();
}

// Check executor throws properly if the same node is added a second time
TYPED_TEST(TestExecutors, addNodeTwoExecutors)
{
  using ExecutorType = TypeParam;
  ExecutorType executor1;
  ExecutorType executor2;
  EXPECT_NO_THROW(executor1.add_node(this->node));
  EXPECT_THROW(executor2.add_node(this->node), std::runtime_error);
  executor1.remove_node(this->node, true);
}

// Check simple spin example
TYPED_TEST(TestExecutors, spinWithTimer)
{
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

TYPED_TEST(TestExecutors, spinWhileAlreadySpinning)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  std::atomic_bool timer_completed = false;
  auto timer = this->node->create_wall_timer(
    1ms, [&]() {
      timer_completed.store(true);
    });

  executor.add_node(this->node);
  std::thread spinner([&]() {executor.spin();});

  // Sleep for a short time to verify executor.spin() is going, and didn't throw.
  auto start = std::chrono::steady_clock::now();
  while (!timer_completed.load() && (std::chrono::steady_clock::now() - start) < 10s) {
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
TYPED_TEST(TestExecutors, testSpinUntilFutureComplete)
{
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
TYPED_TEST(TestExecutors, testSpinUntilSharedFutureComplete)
{
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
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteNoTimeout)
{
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
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteWithTimeout)
{
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

TYPED_TEST(TestExecutors, spinAll)
{
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

// Helper function to convert chrono durations into a scalar that GoogleTest
// can more easily compare and print.
template<typename DurationT>
auto
to_nanoseconds_helper(DurationT duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

// The purpose of this test is to check that the ExecutorT.spin_some() method:
//   - works nominally (it can execute entities)
//   - it can execute multiple items at once
//   - it does not wait for work to be available before returning
TYPED_TEST(TestExecutors, spinSome)
{
  using ExecutorType = TypeParam;

  // Use an isolated callback group to avoid interference from any housekeeping
  // items that may be in the default callback group of the node.
  constexpr bool automatically_add_to_executor_with_node = false;
  auto isolated_callback_group = this->node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    automatically_add_to_executor_with_node);

  // Check that spin_some() returns quickly when there is no work to be done.
  // This can be a false positive if there is somehow some work for the executor
  // to do that has not been considered, but the isolated callback group should
  // avoid that.
  {
    ExecutorType executor;
    executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

    auto start = std::chrono::steady_clock::now();
    // spin_some with some non-trival "max_duration" and check that it does not
    // take anywhere near that long to execute.
    constexpr auto max_duration = 10s;
    executor.spin_some(max_duration);
    EXPECT_LT(
      to_nanoseconds_helper(std::chrono::steady_clock::now() - start),
      to_nanoseconds_helper(max_duration / 2))
      << "spin_some() took a long time to execute when it should have done "
      << "nothing and should not have blocked either, but this could be a "
      << "false negative if the computer is really slow";
  }

  // Check that having one thing ready gets executed by spin_some().
  auto waitable_interfaces = this->node->get_node_waitables_interface();
  auto my_waitable1 = std::make_shared<TestWaitable>();
  waitable_interfaces->add_waitable(my_waitable1, isolated_callback_group);
  {
    ExecutorType executor;
    executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

    my_waitable1->trigger();

    // The long duration should not matter, as executing the waitable is
    // non-blocking, and spin_some() should exit after completing the available
    // work.
    auto start = std::chrono::steady_clock::now();
    constexpr auto max_duration = 10s;
    executor.spin_some(max_duration);
    EXPECT_LT(
      to_nanoseconds_helper(std::chrono::steady_clock::now() - start),
      to_nanoseconds_helper(max_duration / 2))
      << "spin_some() took a long time to execute when it should have very "
      << "little to do and should not have blocked either, but this could be a "
      << "false negative if the computer is really slow";

    EXPECT_EQ(my_waitable1->get_count(), 1u)
      << "spin_some() failed to execute a waitable that was triggered";
  }

  // Check that multiple things being ready are executed by spin_some().
  auto my_waitable2 = std::make_shared<TestWaitable>();
  waitable_interfaces->add_waitable(my_waitable2, isolated_callback_group);
  {
    ExecutorType executor;
    executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

    const size_t original_my_waitable1_count = my_waitable1->get_count();
    my_waitable1->trigger();
    my_waitable2->trigger();

    // The long duration should not matter, as executing the waitable is
    // non-blocking, and spin_some() should exit after completing the available
    // work.
    auto start = std::chrono::steady_clock::now();
    constexpr auto max_duration = 10s;
    executor.spin_some(max_duration);
    EXPECT_LT(
      to_nanoseconds_helper(std::chrono::steady_clock::now() - start),
      to_nanoseconds_helper(max_duration / 2))
      << "spin_some() took a long time to execute when it should have very "
      << "little to do and should not have blocked either, but this could be a "
      << "false negative if the computer is really slow";

    EXPECT_EQ(my_waitable1->get_count(), original_my_waitable1_count + 1)
      << "spin_some() failed to execute a waitable that was triggered";
    EXPECT_EQ(my_waitable2->get_count(), 1u)
      << "spin_some() failed to execute a waitable that was triggered";
  }
}

// The purpose of this test is to check that the ExecutorT.spin_some() method:
//   - does not continue executing after max_duration has elapsed
TYPED_TEST(TestExecutors, spinSomeMaxDuration)
{
  using ExecutorType = TypeParam;

  // TODO(wjwwood): The `StaticSingleThreadedExecutor`
  //   do not properly implement max_duration (it seems), so disable this test
  //   for them in the meantime.
  //   see: https://github.com/ros2/rclcpp/issues/2462
  if (
    std::is_same<ExecutorType, rclcpp::executors::StaticSingleThreadedExecutor>())
  {
    GTEST_SKIP();
  }

  // Use an isolated callback group to avoid interference from any housekeeping
  // items that may be in the default callback group of the node.
  constexpr bool automatically_add_to_executor_with_node = false;
  auto isolated_callback_group = this->node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    automatically_add_to_executor_with_node);

  // Set up a situation with two waitables that take time to execute, such that
  // the time it takes to execute two waitables far exceeds the max_duration
  // given to spin_some(), which should result in spin_some() starting to
  // execute one of them, have the max duration elapse, finish executing one
  // of them, then returning before starting on the second.
  constexpr auto max_duration = 100ms;  // relatively short because we expect to exceed it
  constexpr auto waitable_callback_duration = max_duration * 2;
  auto long_running_callback = [&waitable_callback_duration]() {
      std::this_thread::sleep_for(waitable_callback_duration);
    };

  auto waitable_interfaces = this->node->get_node_waitables_interface();

  auto my_waitable1 = std::make_shared<TestWaitable>();
  my_waitable1->set_on_execute_callback(long_running_callback);
  waitable_interfaces->add_waitable(my_waitable1, isolated_callback_group);

  auto my_waitable2 = std::make_shared<TestWaitable>();
  my_waitable2->set_on_execute_callback(long_running_callback);
  waitable_interfaces->add_waitable(my_waitable2, isolated_callback_group);

  my_waitable1->trigger();
  my_waitable2->trigger();

  ExecutorType executor;
  executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

  auto start = std::chrono::steady_clock::now();
  // spin_some and check that it does not take longer than two of waitable_callback_duration,
  // nor significantly less than a single waitable_callback_duration.
  executor.spin_some(max_duration);
  auto spin_some_run_time = std::chrono::steady_clock::now() - start;
  EXPECT_GT(
    to_nanoseconds_helper(spin_some_run_time),
    to_nanoseconds_helper(waitable_callback_duration / 2))
    << "spin_some() took less than half the expected time to execute a single "
    << "waitable, which implies it did not actually execute one when it was "
    << "expected to";
  EXPECT_LT(
    to_nanoseconds_helper(spin_some_run_time),
    to_nanoseconds_helper(waitable_callback_duration * 2))
    << "spin_some() took longer than expected to execute by a significant margin, but "
    << "this could be a false positive on a very slow computer";

  // check that exactly one of the waitables were executed (do not depend on a specific order)
  size_t number_of_waitables_executed = my_waitable1->get_count() + my_waitable2->get_count();
  EXPECT_EQ(number_of_waitables_executed, 1u)
    << "expected exactly one of the two waitables to be executed, but "
    << "my_waitable1->get_count(): " << my_waitable1->get_count() << " and "
    << "my_waitable2->get_count(): " << my_waitable2->get_count();
}

// Check spin_node_until_future_complete with node base pointer
TYPED_TEST(TestExecutors, testSpinNodeUntilFutureCompleteNodeBasePtr)
{
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
TYPED_TEST(TestExecutors, testSpinNodeUntilFutureCompleteNodePtr)
{
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
TYPED_TEST(TestExecutors, testSpinUntilFutureCompleteInterrupted)
{
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

// This test verifies that the add_node operation is robust wrt race conditions.
// It's mostly meant to prevent regressions in the events-executor, but the operation should be
// thread-safe in all executor implementations.
// The initial implementation of the events-executor contained a bug where the executor
// would end up in an inconsistent state and stop processing interrupt/shutdown notifications.
// Manually adding a node to the executor results in a) producing a notify waitable event
// and b) refreshing the executor collections.
// The inconsistent state would happen if the event was processed before the collections were
// finished to be refreshed: the executor would pick up the event but be unable to process it.
// This would leave the `notify_waitable_event_pushed_` flag to true, preventing additional
// notify waitable events to be pushed.
// The behavior is observable only under heavy load, so this test spawns several worker
// threads. Due to the nature of the bug, this test may still succeed even if the
// bug is present. However repeated runs will show its flakiness nature and indicate
// an eventual regression.
TYPED_TEST(TestExecutors, testRaceConditionAddNode)
{
  using ExecutorType = TypeParam;

  // Spawn some threads to do some heavy work
  std::atomic<bool> should_cancel = false;
  std::vector<std::thread> stress_threads;
  for (size_t i = 0; i < 5 * std::thread::hardware_concurrency(); i++) {
    stress_threads.emplace_back(
      [&should_cancel, i]() {
        // This is just some arbitrary heavy work
        volatile size_t total = 0;
        for (size_t k = 0; k < 549528914167; k++) {
          if (should_cancel) {
            break;
          }
          total += k * (i + 42);
          (void)total;
        }
      });
  }

  // Create an executor
  auto executor = std::make_shared<ExecutorType>();
  // Start spinning
  auto executor_thread = std::thread(
    [executor]() {
      executor->spin();
    });
  // Add a node to the executor
  executor->add_node(this->node);

  // Cancel the executor (make sure that it's already spinning first)
  while (!executor->is_spinning() && rclcpp::ok()) {
    continue;
  }
  executor->cancel();

  // Try to join the thread after cancelling the executor
  // This is the "test". We want to make sure that we can still cancel the executor
  // regardless of the presence of race conditions
  executor_thread.join();

  // The test is now completed: we can join the stress threads
  should_cancel = true;
  for (auto & t : stress_threads) {
    t.join();
  }
}

// Check spin_until_future_complete with node base pointer (instantiates its own executor)
TEST(TestExecutors, testSpinUntilFutureCompleteNodeBasePtr)
{
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
TEST(TestExecutors, testSpinUntilFutureCompleteNodePtr)
{
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

// Check spin functions with non default context
TEST(TestExecutors, testSpinWithNonDefaultContext)
{
  auto non_default_context = std::make_shared<rclcpp::Context>();
  non_default_context->init(0, nullptr);

  {
    auto node =
      std::make_unique<rclcpp::Node>("node", rclcpp::NodeOptions().context(non_default_context));

    EXPECT_NO_THROW(rclcpp::spin_some(node->get_node_base_interface()));

    EXPECT_NO_THROW(rclcpp::spin_all(node->get_node_base_interface(), 1s));

    auto check_spin_until_future_complete = [&]() {
        std::promise<bool> promise;
        std::future<bool> future = promise.get_future();
        promise.set_value(true);

        auto shared_future = future.share();
        auto ret = rclcpp::spin_until_future_complete(
          node->get_node_base_interface(), shared_future, 1s);
        EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
      };
    EXPECT_NO_THROW(check_spin_until_future_complete());
  }

  rclcpp::shutdown(non_default_context);
}

TYPED_TEST(TestExecutors, releaseOwnershipEntityAfterSpinningCancel)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto future = std::async(std::launch::async, [&executor] {executor.spin();});

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr, test_msgs::srv::Empty::Response::SharedPtr) {
    };
  auto server = node->create_service<test_msgs::srv::Empty>("test_service", callback);
  while (!executor.is_spinning()) {
    std::this_thread::sleep_for(50ms);
  }
  executor.add_node(node);
  std::this_thread::sleep_for(50ms);
  executor.cancel();
  std::future_status future_status = future.wait_for(1s);
  EXPECT_EQ(future_status, std::future_status::ready);

  EXPECT_EQ(server.use_count(), 1);
}

TYPED_TEST(TestExecutors, testRaceDropCallbackGroupFromSecondThread)
{
  using ExecutorType = TypeParam;

  // Create an executor
  ExecutorType executor;
  executor.add_node(this->node);

  // Start spinning
  auto executor_thread = std::thread(
    [&executor]() {
      executor.spin();
    });

  // As the problem is a race, we do this multiple times,
  // to raise our chances of hitting the problem
  for (size_t i = 0; i < 10; i++) {
    auto cg = this->node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto timer = this->node->create_timer(1s, [] {}, cg);
    // sleep a bit, so that the spin thread can pick up the callback group
    // and add it to the executor
    std::this_thread::sleep_for(5ms);

    // At this point the callbackgroup should be used within the waitset of the executor
    // as we leave the scope, the reference to cg will be dropped.
    // If the executor has a race, we will experience a segfault at this point.
  }

  executor.cancel();
  executor_thread.join();
}

TYPED_TEST(TestExecutors, dropSomeTimer)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto node = std::make_shared<rclcpp::Node>("test_node");

  bool timer1_works = false;
  bool timer2_works = false;

  auto timer1 = node->create_timer(std::chrono::milliseconds(10), [&timer1_works]() {
        timer1_works = true;
  });
  auto timer2 = node->create_timer(std::chrono::milliseconds(10), [&timer2_works]() {
        timer2_works = true;
  });

  executor.add_node(node);

  // first let's make sure that both timers work
  auto max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!timer1_works || !timer2_works) {
    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  // delete timer 2. Note, the executor uses an unordered map internally, to order
  // the entities added to the rcl waitset therefore the order is kind of undefined,
  // and this test may be flaky. In case it triggers, something is most likely
  // really broken.
  timer2.reset();

  timer1_works = false;
  timer2_works = false;
  max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!timer1_works && !timer2_works) {
    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  ASSERT_TRUE(timer1_works || timer2_works);
}

TYPED_TEST(TestExecutors, dropSomeNodeWithTimer)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto node1 = std::make_shared<rclcpp::Node>("test_node_1");
  auto node2 = std::make_shared<rclcpp::Node>("test_node_2");

  bool timer1_works = false;
  bool timer2_works = false;

  auto timer1 = node1->create_timer(std::chrono::milliseconds(10), [&timer1_works]() {
        timer1_works = true;
  });
  auto timer2 = node2->create_timer(std::chrono::milliseconds(10), [&timer2_works]() {
        timer2_works = true;
  });

  executor.add_node(node1);
  executor.add_node(node2);

  // first let's make sure that both timers work
  auto max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!timer1_works || !timer2_works) {
    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  // delete node 1.
  node1 = nullptr;

  timer2_works = false;
  max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!timer2_works) {
    // let the executor pick up the node and the timer
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  ASSERT_TRUE(timer2_works);
}

TYPED_TEST(TestExecutors, dropSomeSubscription)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto node = std::make_shared<rclcpp::Node>("test_node");

  bool sub1_works = false;
  bool sub2_works = false;

  auto sub1 = node->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub1_works](const test_msgs::msg::Empty &) {
        sub1_works = true;
  });
  auto sub2 = node->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub2_works](const test_msgs::msg::Empty &) {
        sub2_works = true;
  });

  auto pub = node->create_publisher<test_msgs::msg::Empty>("/test_drop", 10);

  executor.add_node(node);

  // first let's make sure that both timers work
  auto max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!sub1_works || !sub2_works) {
    pub->publish(test_msgs::msg::Empty());

    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  // delete subscription 2. Note, the executor uses an unordered map internally, to order
  // the entities added to the rcl waitset therefore the order is kind of undefined,
  // and this test may be flaky. In case it triggers, something is most likely
  // really broken.
  sub2.reset();

  sub1_works = false;
  sub2_works = false;
  max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!sub1_works && !sub2_works) {
    pub->publish(test_msgs::msg::Empty());

    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  ASSERT_TRUE(sub1_works || sub2_works);
}

TYPED_TEST(TestExecutors, dropSomeNodesWithSubscription)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto node1 = std::make_shared<rclcpp::Node>("test_node_1");
  auto node2 = std::make_shared<rclcpp::Node>("test_node_2");

  bool sub1_works = false;
  bool sub2_works = false;

  auto sub1 = node1->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub1_works](const test_msgs::msg::Empty &) {
        sub1_works = true;
  });
  auto sub2 = node2->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub2_works](const test_msgs::msg::Empty &) {
        sub2_works = true;
  });

  auto pub = node->create_publisher<test_msgs::msg::Empty>("/test_drop", 10);

  executor.add_node(node);
  executor.add_node(node1);
  executor.add_node(node2);

  // first let's make sure that both subscribers work
  auto max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!sub1_works || !sub2_works) {
    pub->publish(test_msgs::msg::Empty());

    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  // delete node 2.
  node2 = nullptr;

  sub1_works = false;
  max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while(!sub1_works) {
    pub->publish(test_msgs::msg::Empty());

    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  ASSERT_TRUE(sub1_works);
}

TYPED_TEST(TestExecutors, dropSubscriptionDuringCallback)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;


  auto node = std::make_shared<rclcpp::Node>("test_node");

  bool sub1_works = false;
  bool sub2_works = false;

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  rclcpp::SubscriptionOptions sub_ops;
  sub_ops.callback_group = cbg;

  rclcpp::SubscriptionBase::SharedPtr sub1;
  rclcpp::SubscriptionBase::SharedPtr sub2;

  // Note, the executor uses an unordered map internally, to order
  // the entities added to the rcl waitset therefore the order of the subscriptions
  // is kind of undefined. Therefore each sub deletes the other one.
  sub1 = node->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub1_works, &sub2](const test_msgs::msg::Empty &) {
        sub1_works = true;
        // delete the other subscriber
        sub2.reset();
  }, sub_ops);
  sub2 = node->create_subscription<test_msgs::msg::Empty>("/test_drop", 10,
      [&sub2_works, &sub1](const test_msgs::msg::Empty &) {
        sub2_works = true;
        // delete the other subscriber
        sub1.reset();
  }, sub_ops);

  auto pub = node->create_publisher<test_msgs::msg::Empty>("/test_drop", 10);

  // wait for both subs to be connected
  auto max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
  while ((sub1->get_publisher_count() == 0) || (sub2->get_publisher_count() == 0)) {
    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  executor.add_node(node);

  // publish some messages, until one subscriber fired. As both subscribers are
  // connected to the same topic, they should fire in the same wait.
  max_end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (!sub1_works && !sub2_works) {
    pub->publish(test_msgs::msg::Empty());

    // let the executor pick up the node and the timers
    executor.spin_all(std::chrono::milliseconds(10));

    const auto cur_time = std::chrono::steady_clock::now();
    ASSERT_LT(cur_time, max_end_time);
  }

  // only one subscriber must have worked, as the other
  // one was deleted during the callback
  ASSERT_TRUE(!sub1_works || !sub2_works);
}
