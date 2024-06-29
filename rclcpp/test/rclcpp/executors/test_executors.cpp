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
#include "rcpputils/scope_exit.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "./executor_types.hpp"

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

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable() = default;

  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    if (trigger_count_ > 0) {
      // Keep the gc triggered until the trigger count is reduced back to zero.
      // This is necessary if trigger() results in the wait set waking, but not
      // executing this waitable, in which case it needs to be re-triggered.
      gc_.trigger();
    }
    rclcpp::detail::add_guard_condition_to_rcl_wait_set(wait_set, gc_);
  }

  void trigger()
  {
    trigger_count_++;
    gc_.trigger();
  }

  bool
  is_ready(const rcl_wait_set_t & wait_set) override
  {
    is_ready_count_++;
    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      auto rcl_guard_condition = wait_set.guard_conditions[i];
      if (&gc_.get_rcl_guard_condition() == rcl_guard_condition) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<void>
  take_data() override
  {
    return nullptr;
  }

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void) id;
    return nullptr;
  }

  void
  execute(const std::shared_ptr<void> &) override
  {
    trigger_count_--;
    count_++;
    if (nullptr != on_execute_callback_) {
      on_execute_callback_();
    } else {
      // TODO(wjwwood): I don't know why this was here, but probably it should
      //   not be there, or test cases where that is important should use the
      //   on_execute_callback?
      std::this_thread::sleep_for(3ms);
    }
  }

  void
  set_on_execute_callback(std::function<void()> on_execute_callback)
  {
    on_execute_callback_ = on_execute_callback;
  }

  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    auto gc_callback = [callback](size_t count) {
        callback(count, 0);
      };
    gc_.set_on_trigger_callback(gc_callback);
  }

  void
  clear_on_ready_callback() override
  {
    gc_.set_on_trigger_callback(nullptr);
  }

  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  size_t
  get_count() const
  {
    return count_;
  }

  size_t
  get_is_ready_call_count() const
  {
    return is_ready_count_;
  }

private:
  std::atomic<size_t> trigger_count_ = 0;
  std::atomic<size_t> is_ready_count_ = 0;
  std::atomic<size_t> count_ = 0;
  rclcpp::GuardCondition gc_;
  std::function<void()> on_execute_callback_ = nullptr;
};

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
TYPED_TEST(TestExecutors, spin_some)
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
TYPED_TEST(TestExecutors, spin_some_max_duration)
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
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

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

// Check that executors are correctly notified while they are spinning
// we notify twice to ensure that the notify waitable is still working
// after the first notification
TYPED_TEST(TestExecutors, notifyTwiceWhileSpinning)
{
  using ExecutorType = TypeParam;

  // Create executor, add the node and start spinning
  ExecutorType executor;
  executor.add_node(this->node);
  std::thread spinner([&]() {executor.spin();});

  // Wait for executor to be spinning
  while (!executor.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Create the first subscription while the executor is already spinning
  std::atomic<size_t> sub1_msg_count {0};
  auto sub1 = this->node->template create_subscription<test_msgs::msg::Empty>(
    this->publisher->get_topic_name(),
    rclcpp::QoS(10),
    [&sub1_msg_count](test_msgs::msg::Empty::ConstSharedPtr) {
      sub1_msg_count++;
    });

  // Publish a message and verify it's received
  this->publisher->publish(test_msgs::msg::Empty());
  auto start = std::chrono::steady_clock::now();
  while (sub1_msg_count == 0 && (std::chrono::steady_clock::now() - start) < 10s) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(sub1_msg_count, 1u);

  // Create a second subscription while the executor is already spinning
  std::atomic<size_t> sub2_msg_count {0};
  auto sub2 = this->node->template create_subscription<test_msgs::msg::Empty>(
    this->publisher->get_topic_name(),
    rclcpp::QoS(10),
    [&sub2_msg_count](test_msgs::msg::Empty::ConstSharedPtr) {
      sub2_msg_count++;
    });

  // Publish a message and verify it's received by both subscriptions
  this->publisher->publish(test_msgs::msg::Empty());
  start = std::chrono::steady_clock::now();
  while (
    sub1_msg_count == 1 &&
    sub2_msg_count == 0 &&
    (std::chrono::steady_clock::now() - start) < 10s)
  {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(sub1_msg_count, 2u);
  EXPECT_EQ(sub2_msg_count, 1u);

  // Cancel needs to be called before join, so that executor.spin() returns.
  executor.cancel();
  spinner.join();
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

// The purpose of this test is to check that the order of callbacks happen
// in some relation to the order of events and the order in which the callbacks
// were registered.
// This is not a guarantee of executor API, but it is a bit of UB that some
// have come to depend on, see:
//
//   https://github.com/ros2/rclcpp/issues/2532
//
// It should not be changed unless there's a good reason for it (users find it
// the least surprising outcome even if it is not guaranteed), but if there
// is a good reason for changing it, then the executors effected can be skipped,
// or the test can be removed.
// The purpose of this test is to catch this regressions and let the authors of
// the change read up on the above context and act accordingly.
TYPED_TEST(TestExecutors, deterministic_execution_order_ub)
{
  using ExecutorType = TypeParam;

  // number of each entity to test
  constexpr size_t number_of_entities = 20;
  std::vector<size_t> forward(number_of_entities);
  std::iota(std::begin(forward), std::end(forward), 0);
  std::vector<size_t> reverse(number_of_entities);
  std::reverse_copy(std::begin(forward), std::end(forward), std::begin(reverse));

  // The expected results vary based on the registration order (always 0..N-1),
  // the call order (what this means varies based on the entity type), the
  // entity types, and in some cases the executor type.
  // It is also possible that the rmw implementation can play a role in the
  // ordering, depending on how the executor uses the rmw layer.
  // The follow structure and logic tries to capture these details.
  // Each test case represents a case-entity pair,
  // e.g. "forward call order for waitables" or "reverse call order for timers"
  struct test_case
  {
    // If this is true, then the test case should be skipped.
    bool should_skip;
    // Order in which to invoke the entities, where that is possible to control.
    // For example, the order in which we trigger() the waitables, or the
    // order in which we set the timers up to execute (using increasing periods).
    std::vector<size_t> call_order;
    // Order in which we expect the entities to be executed by the executor.
    std::vector<size_t> expected_execution_order;
  };
  // tests cases are "test_name: {"entity type": {skip, call_order, expected_execution_order}"
  std::map<std::string, std::map<std::string, test_case>> test_cases = {
    {
      "forward call order",
      {
        {"waitable", {false, forward, forward}},
        {"subscription", {false, forward, forward}},
        {"service", {false, forward, forward}},
        {"timer", {false, forward, forward}}
      }
    },
    {
      "reverse call order",
      {
        {"waitable", {false, reverse, forward}},
        {"subscription", {false, reverse, forward}},
        {"service", {false, reverse, forward}},
        // timers are always called in order of which expires soonest, so
        // the registration order doesn't necessarily affect them
        {"timer", {false, reverse, reverse}}
      }
    },
  };

  // Note use this to exclude or modify expected results for executors if this
  // undefined behavior doesn't hold for them:
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>())
  {
    // for the EventsExecutor the call order is the execution order because it
    // tracks the individual events (triggers in the case of waitables) and
    // executes in that order
    test_cases["reverse call order"]["waitable"] = {false, reverse, reverse};
    // timers are unaffected by the above about waitables, as they are always
    // executed in "call order" even in the other executors
    // but, subscription and service execution order is driven by the rmw impl
    // due to how the EventsExecutor uses the rmw interface, so we'll skip those
    for (auto & test_case_pair : test_cases) {
      for (auto & entity_test_case_pair : test_case_pair.second) {
        if (
          entity_test_case_pair.first == "subscription" ||
          entity_test_case_pair.first == "service")
        {
          entity_test_case_pair.second = {true, {}, {}};
        }
      }
    }
  }

  // Set up a situation with N waitables, added in order (1, ..., N) and then
  // trigger them in various orders between calls to spin, to see that the order
  // is impacted by the registration order (in most cases).
  // Note that we always add/register, trigger, then wait/spin, because this
  // undefined behavior related to execution order only applies to entities
  // that were "ready" in between calls to spin, i.e. they appear to become
  // "ready" to the executor at the "same time".
  // Also note, that this ordering only applies within entities of the same type
  // as well, there are other parts of the executor that determine the order
  // between entity types, e.g. the default scheduling (at the time of writing)
  // prefers timers, then subscriptions, then service servers, then service
  // clients, and then waitables, see: Executor::get_next_ready_executable()
  // But that might be different for different executors and may change in the
  // future.
  // So here we just test order withing a few different waitable instances only.
  // Further down we test similar set ups with other entities like subscriptions
  // and timers.

  constexpr bool automatically_add_to_executor_with_node = false;
  auto isolated_callback_group = this->node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    automatically_add_to_executor_with_node);

  // perform each of the test cases for waitables
  {
    auto waitable_interfaces = this->node->get_node_waitables_interface();

    std::vector<std::shared_ptr<TestWaitable>> waitables;
    for (size_t i = 0; i < number_of_entities; ++i) {
      auto my_waitable = std::make_shared<TestWaitable>();
      waitable_interfaces->add_waitable(my_waitable, isolated_callback_group);
      waitables.push_back(my_waitable);
    }

    for (const auto & test_case_pair : test_cases) {
      const std::string & test_case_name = test_case_pair.first;
      const auto & test_case = test_case_pair.second.at("waitable");
      if (test_case.should_skip) {
        continue;
      }

      ExecutorType executor;
      executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

      RCPPUTILS_SCOPE_EXIT({
        for (size_t i = 0; i < number_of_entities; ++i) {
          waitables[i]->set_on_execute_callback(nullptr);
        }
      });

      std::vector<size_t> actual_order;
      for (size_t i : test_case.call_order) {
        waitables[i]->set_on_execute_callback([&actual_order, i]() {actual_order.push_back(i);});
        waitables[i]->trigger();
      }

      while (actual_order.size() < number_of_entities && rclcpp::ok()) {
        executor.spin_once(10s);  // large timeout because it should normally exit quickly
      }

      EXPECT_EQ(actual_order, test_case.expected_execution_order)
        << "callback call order of waitables in test case '" << test_case_name
        << "' different than expected, this may be a false positive, see test "
        << "description";
    }
  }

  // perform each of the test cases for subscriptions
  {
    const std::string test_topic_name = "~/deterministic_execution_order_ub";
    std::map<rclcpp::SubscriptionBase *, std::function<void()>> on_sub_data_callbacks;
    std::vector<rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr> subscriptions;
    rclcpp::SubscriptionOptions so;
    so.callback_group = isolated_callback_group;
    for (size_t i = 0; i < number_of_entities; ++i) {
      size_t next_sub_index = subscriptions.size();
      auto sub = this->node->template create_subscription<test_msgs::msg::Empty>(
        test_topic_name,
        10,
        [&on_sub_data_callbacks, &subscriptions, next_sub_index](const test_msgs::msg::Empty &) {
          auto this_sub_pointer = subscriptions[next_sub_index].get();
          auto callback_for_sub_it = on_sub_data_callbacks.find(this_sub_pointer);
          ASSERT_NE(callback_for_sub_it, on_sub_data_callbacks.end());
          auto on_sub_data_callback = callback_for_sub_it->second;
          if (on_sub_data_callback) {
            on_sub_data_callback();
          }
        },
        so);
      subscriptions.push_back(sub);
    }

    for (const auto & test_case_pair : test_cases) {
      const std::string & test_case_name = test_case_pair.first;
      const auto & test_case = test_case_pair.second.at("subscription");
      if (test_case.should_skip) {
        continue;
      }

      ExecutorType executor;
      executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

      RCPPUTILS_SCOPE_EXIT({
        on_sub_data_callbacks.clear();
      });

      std::vector<size_t> actual_order;
      for (size_t i = 0; i < number_of_entities; ++i) {
        auto sub = subscriptions[i];
        on_sub_data_callbacks[sub.get()] = [&actual_order, i]() {
            actual_order.push_back(i);
          };
      }

      // create publisher and wait for all of the subscriptions to match
      auto pub = this->node->template create_publisher<test_msgs::msg::Empty>(test_topic_name, 10);
      size_t number_of_matches = pub->get_subscription_count();
      while (number_of_matches < number_of_entities && rclcpp::ok()) {
        executor.spin_once(10s);  // large timeout because it should normally exit quickly
        number_of_matches = pub->get_subscription_count();
      }

      // publish once and wait for all subscriptions to be handled
      pub->publish(test_msgs::msg::Empty());
      while (actual_order.size() < number_of_entities && rclcpp::ok()) {
        executor.spin_once(10s);  // large timeout because it should normally exit quickly
      }

      EXPECT_EQ(actual_order, test_case.expected_execution_order)
        << "callback call order of subscriptions in test case '" << test_case_name
        << "' different than expected, this may be a false positive, see test "
        << "description";
    }
  }

  // perform each of the test cases for service servers
  {
    const std::string test_service_name = "~/deterministic_execution_order_ub";
    std::map<rclcpp::ServiceBase *, std::function<void()>> on_request_callbacks;
    std::vector<rclcpp::Service<test_msgs::srv::Empty>::SharedPtr> services;
    std::vector<rclcpp::Client<test_msgs::srv::Empty>::SharedPtr> clients;
    for (size_t i = 0; i < number_of_entities; ++i) {
      size_t next_srv_index = services.size();
      auto srv = this->node->template create_service<test_msgs::srv::Empty>(
        test_service_name + "_" + std::to_string(i),
        [&on_request_callbacks, &services, next_srv_index](
          std::shared_ptr<test_msgs::srv::Empty::Request>,
          std::shared_ptr<test_msgs::srv::Empty::Response>
        ) {
          auto this_srv_pointer = services[next_srv_index].get();
          auto callback_for_srv_it = on_request_callbacks.find(this_srv_pointer);
          ASSERT_NE(callback_for_srv_it, on_request_callbacks.end());
          auto on_request_callback = callback_for_srv_it->second;
          if (on_request_callback) {
            on_request_callback();
          }
        },
        10,
        isolated_callback_group);
      services.push_back(srv);
      auto client = this->node->template create_client<test_msgs::srv::Empty>(
        test_service_name + "_" + std::to_string(i),
        10,
        isolated_callback_group
      );
      clients.push_back(client);
    }

    for (const auto & test_case_pair : test_cases) {
      const std::string & test_case_name = test_case_pair.first;
      const auto & test_case = test_case_pair.second.at("service");
      if (test_case.should_skip) {
        continue;
      }

      ExecutorType executor;
      executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

      RCPPUTILS_SCOPE_EXIT({
        on_request_callbacks.clear();
      });

      std::vector<size_t> actual_order;
      for (size_t i = 0; i < number_of_entities; ++i) {
        auto srv = services[i];
        on_request_callbacks[srv.get()] = [&actual_order, i]() {
            actual_order.push_back(i);
          };
      }

      // wait for all of the services to match
      for (const auto & client : clients) {
        bool matched = client->wait_for_service(10s);  // long timeout, but should be quick
        ASSERT_TRUE(matched);
      }

      // send requests in order
      for (size_t i : test_case.call_order) {
        clients[i]->async_send_request(std::make_shared<test_msgs::srv::Empty::Request>());
      }

      // wait for all the requests to be handled
      while (actual_order.size() < number_of_entities && rclcpp::ok()) {
        executor.spin_once(10s);  // large timeout because it should normally exit quickly
      }

      EXPECT_EQ(actual_order, test_case.expected_execution_order)
        << "callback call order of service servers in test case '" << test_case_name
        << "' different than expected, this may be a false positive, see test "
        << "description";
    }
  }

  // perform each of the test cases for timers
  {
    for (const auto & test_case_pair : test_cases) {
      const std::string & test_case_name = test_case_pair.first;
      const auto & test_case = test_case_pair.second.at("timer");
      if (test_case.should_skip) {
        continue;
      }

      std::map<rclcpp::TimerBase *, std::function<void()>> timer_callbacks;
      std::vector<rclcpp::TimerBase::SharedPtr> timers;
      for (size_t i = 0; i < number_of_entities; ++i) {
        // "call order" for timers will be simulated by setting them at different
        // periods, with the "first" ones having the smallest period.
        auto period = 1ms + std::chrono::milliseconds(test_case.call_order[i]);
        auto timer = this->node->create_timer(
          period,
          [&timer_callbacks](rclcpp::TimerBase & timer) {
            auto timer_callback_it = timer_callbacks.find(&timer);
            ASSERT_NE(timer_callback_it, timer_callbacks.end());
            if (nullptr != timer_callback_it->second) {
              timer_callback_it->second();
            }
          },
          isolated_callback_group);
        timers.push_back(timer);
      }

      ExecutorType executor;
      executor.add_callback_group(isolated_callback_group, this->node->get_node_base_interface());

      RCPPUTILS_SCOPE_EXIT({
        timer_callbacks.clear();
      });

      std::vector<size_t> actual_order;
      for (size_t i = 0; i < number_of_entities; ++i) {
        ASSERT_LT(i, timers.size());
        auto & timer = timers[i];
        timer_callbacks[timer.get()] = [&actual_order, &timer, i]() {
            actual_order.push_back(i);
            // only allow execution once
            timer->cancel();
          };
      }

      while (actual_order.size() < number_of_entities && rclcpp::ok()) {
        executor.spin_once(10s);  // large timeout because it should normally exit quickly
      }

      EXPECT_EQ(actual_order, test_case.expected_execution_order)
        << "callback call order of timers in test case '" << test_case_name
        << "' different than expected, this may be a false positive, see test "
        << "description";
    }
  }
}

template<typename T>
class TestBusyWaiting : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();
    node = std::make_shared<rclcpp::Node>("node", test_name.str());
    callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      /* automatically_add_to_executor_with_node =*/ false);

    auto waitable_interfaces = node->get_node_waitables_interface();
    waitable = std::make_shared<TestWaitable>();
    waitable_interfaces->add_waitable(waitable, callback_group);

    executor = std::make_shared<T>();
    executor->add_callback_group(callback_group, node->get_node_base_interface());
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void
  set_up_and_trigger_waitable(std::function<void()> extra_callback = nullptr)
  {
    this->has_executed = false;
    this->waitable->set_on_execute_callback([this, extra_callback]() {
        if (!this->has_executed) {
        // trigger once to see if the second trigger is handled or not
        // this follow up trigger simulates new entities becoming ready while
        // the executor is executing something else, e.g. subscription got data
        // or a timer expired, etc.
        // spin_some would not handle this second trigger, since it collects
        // work only once, whereas spin_all should handle it since it
        // collects work multiple times
          this->waitable->trigger();
          this->has_executed = true;
        }
        if (nullptr != extra_callback) {
          extra_callback();
        }
    });
    this->waitable->trigger();
  }

  void
  check_for_busy_waits(std::chrono::steady_clock::time_point start_time)
  {
    // rough time based check, since the work to be done was very small it
    // should be safe to check that we didn't use more than half the
    // max duration, which itself is much larger than necessary
    // however, it could still produce a false-positive
    EXPECT_LT(
      std::chrono::steady_clock::now() - start_time,
      max_duration / 2)
      << "executor took a long time to execute when it should have done "
      << "nothing and should not have blocked either, but this could be a "
      << "false negative if the computer is really slow";

    // this check is making some assumptions about the implementation of the
    // executors, but it should be safe to say that a busy wait may result in
    // hundreds or thousands of calls to is_ready(), but "normal" executor
    // behavior should be within an order of magnitude of the number of
    // times that the waitable was executed
    ASSERT_LT(waitable->get_is_ready_call_count(), 10u * this->waitable->get_count());
  }

  static constexpr auto max_duration = 10s;

  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::shared_ptr<TestWaitable> waitable;
  std::chrono::steady_clock::time_point start_time;
  std::shared_ptr<T> executor;
  bool has_executed;
};

TYPED_TEST_SUITE(TestBusyWaiting, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestBusyWaiting, test_spin_all)
{
  this->set_up_and_trigger_waitable();

  auto start_time = std::chrono::steady_clock::now();
  this->executor->spin_all(this->max_duration);
  this->check_for_busy_waits(start_time);
  // this should get the initial trigger, and the follow up from in the callback
  ASSERT_EQ(this->waitable->get_count(), 2u);
}

TYPED_TEST(TestBusyWaiting, test_spin_some)
{
  this->set_up_and_trigger_waitable();

  auto start_time = std::chrono::steady_clock::now();
  this->executor->spin_some(this->max_duration);
  this->check_for_busy_waits(start_time);
  // this should get the inital trigger, but not the follow up in the callback
  ASSERT_EQ(this->waitable->get_count(), 1u);
}

TYPED_TEST(TestBusyWaiting, test_spin)
{
  std::condition_variable cv;
  std::mutex cv_m;
  bool first_check_passed = false;

  this->set_up_and_trigger_waitable([&cv, &cv_m, &first_check_passed]() {
      cv.notify_one();
      if (!first_check_passed) {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait_for(lk, 1s, [&]() {return first_check_passed;});
      }
  });

  auto start_time = std::chrono::steady_clock::now();
  std::thread t([this]() {
      this->executor->spin();
    });

  // wait until thread has started (first execute of waitable)
  {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait_for(lk, 10s);
  }
  EXPECT_GT(this->waitable->get_count(), 0u);

  first_check_passed = true;
  cv.notify_one();

  // wait until the executor has finished (second execute of waitable)
  {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait_for(lk, 10s);
  }
  EXPECT_EQ(this->waitable->get_count(), 2u);

  this->executor->cancel();
  t.join();

  this->check_for_busy_waits(start_time);
  // this should get the initial trigger, and the follow up from in the callback
  ASSERT_EQ(this->waitable->get_count(), 2u);
}
