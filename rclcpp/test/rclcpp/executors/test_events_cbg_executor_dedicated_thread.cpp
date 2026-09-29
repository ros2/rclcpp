// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class TestEventsCBGExecutorDedicatedThread : public testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}
};

/*
 * Test that all callbacks of a callback group with a dedicated thread are
 * executed by exactly one thread, that this thread is the one the
 * thread_init_callback was executed on, that this thread executes no
 * callbacks of other callback groups, and that the name and cpu_affinity
 * settings of DedicatedThreadOptions are applied.
 */
TEST_F(TestEventsCBGExecutorDedicatedThread, dedicated_group_runs_on_own_thread)
{
  auto node = std::make_shared<rclcpp::Node>("test_dedicated_thread");

  auto dedicated_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  std::mutex ids_mutex;
  std::set<std::thread::id> dedicated_ids;
  std::set<std::thread::id> pool_ids;
  std::atomic<size_t> dedicated_count{0};
  std::atomic<size_t> pool_count{0};

  std::string observed_thread_name;
  std::atomic<int> observed_cpu{-1};

  rclcpp::SubscriptionOptions dedicated_opts;
  dedicated_opts.callback_group = dedicated_cbg;

  auto sub_dedicated = node->create_subscription<test_msgs::msg::Empty>(
    "dedicated_topic", 10,
    [&](test_msgs::msg::Empty) {
      {
        std::lock_guard<std::mutex> lock(ids_mutex);
        dedicated_ids.insert(std::this_thread::get_id());
#ifdef __linux__
        char thread_name[16] = {};
        pthread_getname_np(pthread_self(), thread_name, sizeof(thread_name));
        observed_thread_name = thread_name;
        observed_cpu.store(sched_getcpu());
#endif
      }
      dedicated_count++;
    },
    dedicated_opts);

  // uses the default callback group of the node, executed by the worker pool
  auto sub_pool = node->create_subscription<test_msgs::msg::Empty>(
    "pool_topic", 10,
    [&](test_msgs::msg::Empty) {
      {
        std::lock_guard<std::mutex> lock(ids_mutex);
        pool_ids.insert(std::this_thread::get_id());
      }
      pool_count++;
    });

  auto pub_dedicated = node->create_publisher<test_msgs::msg::Empty>("dedicated_topic", 10);
  auto pub_pool = node->create_publisher<test_msgs::msg::Empty>("pool_topic", 10);

  std::atomic<std::thread::id> init_thread_id{};
  std::atomic<size_t> init_calls{0};

  rclcpp::executors::EventsCBGExecutor executor(rclcpp::ExecutorOptions(), 2u);

  rclcpp::executors::EventsCBGExecutor::DedicatedThreadOptions thread_options;
  thread_options.name = "dedicated_test";
  thread_options.cpu_affinity = {0};
  thread_options.thread_init_callback = [&]() {
      init_thread_id.store(std::this_thread::get_id());
      init_calls++;
    };
  executor.set_dedicated_thread_for_callback_group(dedicated_cbg, thread_options);
  executor.add_node(node);

  std::thread spin_thread([&executor]() {executor.spin();});

  test_msgs::msg::Empty msg;
  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline &&
    (dedicated_count.load() < 5 || pool_count.load() < 5))
  {
    pub_dedicated->publish(msg);
    pub_pool->publish(msg);
    std::this_thread::sleep_for(50ms);
  }

  executor.cancel();
  spin_thread.join();

  EXPECT_GE(dedicated_count.load(), 5u) << "dedicated subscription barely ran";
  EXPECT_GE(pool_count.load(), 5u) << "pool subscription barely ran";
  EXPECT_EQ(init_calls.load(), 1u);

  std::lock_guard<std::mutex> lock(ids_mutex);
  ASSERT_EQ(dedicated_ids.size(), 1u) <<
    "callbacks of the dedicated callback group ran on more than one thread";
  const std::thread::id dedicated_id = *dedicated_ids.begin();
  EXPECT_EQ(dedicated_id, init_thread_id.load()) <<
    "thread_init_callback did not run on the dedicated thread";
  EXPECT_EQ(pool_ids.count(dedicated_id), 0u) <<
    "the dedicated thread also executed callbacks of other callback groups";
  EXPECT_NE(dedicated_id, std::this_thread::get_id());

#ifdef __linux__
  EXPECT_EQ(observed_thread_name, "dedicated_test") <<
    "the name of DedicatedThreadOptions was not applied";
  EXPECT_EQ(observed_cpu.load(), 0) <<
    "the cpu_affinity of DedicatedThreadOptions was not applied";
#endif
}

/*
 * Test that timers of a dedicated callback group are executed on the
 * dedicated thread.
 */
TEST_F(TestEventsCBGExecutorDedicatedThread, timer_runs_on_dedicated_thread)
{
  auto node = std::make_shared<rclcpp::Node>("test_dedicated_thread_timer");

  auto dedicated_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  std::mutex ids_mutex;
  std::set<std::thread::id> timer_ids;
  std::atomic<size_t> timer_count{0};

  auto timer = node->create_wall_timer(
    10ms,
    [&]() {
      {
        std::lock_guard<std::mutex> lock(ids_mutex);
        timer_ids.insert(std::this_thread::get_id());
      }
      timer_count++;
    },
    dedicated_cbg);

  std::atomic<std::thread::id> init_thread_id{};

  rclcpp::executors::EventsCBGExecutor executor(rclcpp::ExecutorOptions(), 2u);

  rclcpp::executors::EventsCBGExecutor::DedicatedThreadOptions thread_options;
  thread_options.thread_init_callback = [&]() {
      init_thread_id.store(std::this_thread::get_id());
    };
  executor.set_dedicated_thread_for_callback_group(dedicated_cbg, thread_options);
  executor.add_node(node);

  std::thread spin_thread([&executor]() {executor.spin();});

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline && timer_count.load() < 5) {
    std::this_thread::sleep_for(10ms);
  }

  executor.cancel();
  spin_thread.join();

  EXPECT_GE(timer_count.load(), 5u) << "timer barely fired";

  std::lock_guard<std::mutex> lock(ids_mutex);
  ASSERT_EQ(timer_ids.size(), 1u) <<
    "timer callbacks of the dedicated callback group ran on more than one thread";
  EXPECT_EQ(*timer_ids.begin(), init_thread_id.load()) <<
    "timer did not run on the dedicated thread";
}

/*
 * Test that configuring a dedicated thread for a callback group that is
 * already known to the executor throws.
 */
TEST_F(TestEventsCBGExecutorDedicatedThread, set_dedicated_thread_after_add_throws)
{
  auto node = std::make_shared<rclcpp::Node>("test_dedicated_thread_throws");

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::executors::EventsCBGExecutor executor(rclcpp::ExecutorOptions(), 2u);
  executor.add_node(node);

  EXPECT_THROW(
    executor.set_dedicated_thread_for_callback_group(cbg),
    std::runtime_error);
}

/*
 * Test that an exception thrown in a callback of a dedicated callback group
 * is passed to the exception handler of spin(exception_handler), and that
 * the dedicated thread continues processing events afterwards.
 */
TEST_F(TestEventsCBGExecutorDedicatedThread, exception_handler_used_for_dedicated_thread)
{
  auto node = std::make_shared<rclcpp::Node>("test_dedicated_thread_exception");

  auto dedicated_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  std::atomic<size_t> callback_count{0};

  rclcpp::SubscriptionOptions dedicated_opts;
  dedicated_opts.callback_group = dedicated_cbg;

  auto sub = node->create_subscription<test_msgs::msg::Empty>(
    "dedicated_topic", 10,
    [&](test_msgs::msg::Empty) {
      if (callback_count.fetch_add(1) == 0) {
        throw std::runtime_error("first callback throws");
      }
    },
    dedicated_opts);

  auto pub = node->create_publisher<test_msgs::msg::Empty>("dedicated_topic", 10);

  std::atomic<size_t> handled_exceptions{0};

  rclcpp::executors::EventsCBGExecutor executor(rclcpp::ExecutorOptions(), 2u);
  executor.set_dedicated_thread_for_callback_group(dedicated_cbg);
  executor.add_node(node);

  std::thread spin_thread(
    [&executor, &handled_exceptions]() {
      executor.spin(
        [&handled_exceptions](const std::exception &) {
          handled_exceptions++;
        });
    });

  test_msgs::msg::Empty msg;
  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline && callback_count.load() < 3) {
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }

  executor.cancel();
  spin_thread.join();

  EXPECT_GE(callback_count.load(), 3u) <<
    "dedicated thread stopped processing events after the exception";
  EXPECT_EQ(handled_exceptions.load(), 1u);
}
