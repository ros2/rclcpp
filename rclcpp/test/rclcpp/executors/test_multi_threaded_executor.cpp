// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <future>
#include <string>
#include <memory>
#include <utility>

#include "rcpputils/thread.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class TestMultiThreadedExecutor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

constexpr std::chrono::milliseconds PERIOD_MS = 1000ms;
constexpr double PERIOD = PERIOD_MS.count() / 1000.0;
constexpr double TOLERANCE = PERIOD / 4.0;

/*
   Test that timers are not taken multiple times when using reentrant callback groups.
 */
TEST_F(TestMultiThreadedExecutor, timer_over_take) {
#ifdef __linux__
  // This seems to be the most effective way to force the bug to happen on Linux.
  // This is unnecessary on MacOS, since the default scheduler causes it.
  struct sched_param param;
  param.sched_priority = 0;
  if (sched_setscheduler(0, SCHED_BATCH, &param) != 0) {
    perror("sched_setscheduler");
  }
#endif

  bool yield_before_execute = true;

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2u, yield_before_execute);

  ASSERT_GT(executor.get_number_of_threads(), 1u);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_multi_threaded_executor_timer_over_take");

  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::Clock system_clock(RCL_STEADY_TIME);
  std::mutex last_mutex;
  auto last = system_clock.now();

  std::atomic_int timer_count {0};

  auto timer_callback = [&timer_count, &executor, &system_clock, &last_mutex, &last]() {
      // While this tolerance is a little wide, if the bug occurs, the next step will
      // happen almost instantly. The purpose of this test is not to measure the jitter
      // in timers, just assert that a reasonable amount of time has passed.
      rclcpp::Time now = system_clock.now();
      timer_count++;

      if (timer_count > 5) {
        executor.cancel();
      }

      {
        std::lock_guard<std::mutex> lock(last_mutex);
        double diff = static_cast<double>(std::abs((now - last).nanoseconds())) / 1.0e9;
        last = now;

        if (diff < PERIOD - TOLERANCE) {
          executor.cancel();
          ASSERT_GT(diff, PERIOD - TOLERANCE);
        }
      }
    };

  auto timer = node->create_wall_timer(PERIOD_MS, timer_callback, cbg);
  executor.add_node(node);
  executor.spin();
}

TEST_F(TestMultiThreadedExecutor, thread_attribute_apply) {
  using rcpputils::Thread;
  using rcpputils::ThreadAttribute;
  using rcpputils::ThreadId;
  using rcpputils::SchedPolicy;

  ThreadAttribute attr;
  unsigned thread_count = 10;

#if __linux__
  ThreadId parent_thread = rcpputils::this_thread::get_id();
  {
    sched_param param = {0};
    int r = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    EXPECT_EQ(0, r);
    // SCHED_FIFO, SCHED_RR are require the privilege on linux
    attr.set_sched_policy(SchedPolicy::batch);
  }
#endif

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_count, attr);

  std::atomic<unsigned> succ_count = 0;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
#if __linux__
      int policy;
      sched_param param;
      ThreadId worker_thread = rcpputils::this_thread::get_id();
      int r = pthread_getschedparam(pthread_self(), &policy, &param);
      SchedPolicy rclcpp_policy = SchedPolicy(0x8000'0000 | policy);
      EXPECT_EQ(0, r);
      EXPECT_EQ(SchedPolicy::batch, rclcpp_policy);
      EXPECT_NE(worker_thread, parent_thread);
#endif
      unsigned n = succ_count.fetch_add(1);
      if (n == thread_count - 1) {
        executor.cancel();
      }
    });

  executor.add_node(node);
  executor.spin();
  EXPECT_EQ(succ_count, thread_count);
}

constexpr char const * argv[] = {
  "test_multi_threaded_executor",
  "--ros-args",
  "--thread-attrs-value",
  R"(
- name: RCLCPP_EXECUTOR_MULTI_THREADED
  scheduling_policy: FIFO
  priority: 10
  core_affinity: []
- name: executor-1
  scheduling_policy: RR
  priority: 20
  core_affinity: [0]
  )",
  NULL,
};
constexpr int argc = sizeof(argv) / sizeof(*argv) - 1;

class TestMultiThreadedExecutorAttribute : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(argc, argv);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestMultiThreadedExecutorAttribute, thread_attribute_default_name) {
  using rcpputils::SchedPolicy;
  rclcpp::executors::MultiThreadedExecutor executor;

  const auto & opt_attr = executor.get_thread_attribute();
  ASSERT_TRUE(opt_attr);

  const auto & attr = opt_attr.value();
  EXPECT_EQ(SchedPolicy::fifo, attr.get_sched_policy());
  EXPECT_EQ(10, attr.get_priority());

  const auto & affinity = attr.get_affinity();
  EXPECT_EQ(0, affinity.count());
}

TEST_F(TestMultiThreadedExecutorAttribute, thread_attribute_specified_name) {
  using rcpputils::SchedPolicy;
  auto options = rclcpp::ExecutorOptions();
  options.name = "executor-1";
  rclcpp::executors::MultiThreadedExecutor executor(options);

  const auto & opt_attr = executor.get_thread_attribute();
  ASSERT_TRUE(opt_attr);

  const auto & attr = opt_attr.value();
  EXPECT_EQ(SchedPolicy::rr, attr.get_sched_policy());
  EXPECT_EQ(20, attr.get_priority());

  const auto & affinity = attr.get_affinity();
  EXPECT_EQ(1, affinity.count());
  EXPECT_TRUE(affinity.is_set(0));
}

TEST_F(TestMultiThreadedExecutorAttribute, thread_attribute_not_found) {
  using rcpputils::SchedPolicy;
  auto options = rclcpp::ExecutorOptions();
  options.name = "executor-not-exists";
  rclcpp::executors::MultiThreadedExecutor executor(options);

  const auto & opt_attr = executor.get_thread_attribute();
  ASSERT_FALSE(opt_attr);
}
