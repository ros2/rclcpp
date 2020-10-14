// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executors/timers_manager.hpp"

using namespace std::chrono_literals;

using rclcpp::executors::TimersManager;

using CallbackT = std::function<void ()>;
using TimerT = rclcpp::WallTimer<CallbackT>;

class TestTimersManager : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestTimersManager, empty_manager)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  EXPECT_EQ(TimersManager::MAX_TIME, timers_manager->get_head_timeout());
  EXPECT_FALSE(timers_manager->execute_head_timer());
  EXPECT_NO_THROW(timers_manager->execute_ready_timers());
  EXPECT_NO_THROW(timers_manager->clear_all());
  EXPECT_NO_THROW(timers_manager->start());
  EXPECT_NO_THROW(timers_manager->stop());
}

TEST_F(TestTimersManager, max_time)
{
  // Timers manager max time should be a very big duration that does not cause overflow

  EXPECT_LT(std::chrono::hours(50).count(), TimersManager::MAX_TIME.count());
  auto tp = std::chrono::steady_clock::now() + TimersManager::MAX_TIME;
  EXPECT_TRUE(std::chrono::steady_clock::now() < tp);
}

TEST_F(TestTimersManager, add_run_remove_timer)
{
  size_t t_runs = 0;
  auto t = TimerT::make_shared(
    1ms,
    [&t_runs]() {
      t_runs++;
    },
    rclcpp::contexts::get_global_default_context());
  std::weak_ptr<TimerT> t_weak = t;

  // Add the timer to the timers manager
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());
  timers_manager->add_timer(std::move(t));

  // Sleep for more 3 times the timer period
  std::this_thread::sleep_for(3ms);

  // The timer is executed only once, even if we slept 3 times the period
  timers_manager->execute_ready_timers();
  EXPECT_EQ(1u, t_runs);

  // The timer is still valid after execution
  auto t_tmp = t_weak.lock();
  EXPECT_TRUE(t_tmp != nullptr);

  // Remove the timer from the manager
  timers_manager->remove_timer(std::move(t_tmp));

  // The timer is now not valid anymore
  t_tmp = t_weak.lock();
  EXPECT_FALSE(t_tmp != nullptr);
}

TEST_F(TestTimersManager, clear_all)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  auto t1 = TimerT::make_shared(1ms, CallbackT(), rclcpp::contexts::get_global_default_context());
  std::weak_ptr<TimerT> t1_weak = t1;
  auto t2 = TimerT::make_shared(1ms, CallbackT(), rclcpp::contexts::get_global_default_context());
  std::weak_ptr<TimerT> t2_weak = t2;

  timers_manager->add_timer(std::move(t1));
  timers_manager->add_timer(std::move(t2));

  EXPECT_TRUE(t1_weak.lock() != nullptr);
  EXPECT_TRUE(t2_weak.lock() != nullptr);

  timers_manager->clear_all();

  EXPECT_FALSE(t1_weak.lock() != nullptr);
  EXPECT_FALSE(t2_weak.lock() != nullptr);
}

TEST_F(TestTimersManager, remove_not_existing_timer)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  // Try to remove a nullptr timer
  EXPECT_NO_THROW(timers_manager->remove_timer(nullptr));

  auto t = TimerT::make_shared(1ms, CallbackT(), rclcpp::contexts::get_global_default_context());
  timers_manager->add_timer(t);

  // Remove twice the same timer
  timers_manager->remove_timer(t);
  EXPECT_NO_THROW(timers_manager->remove_timer(t));
}

TEST_F(TestTimersManager, timers_thread_exclusive_usage)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  timers_manager->start();

  EXPECT_THROW(timers_manager->get_head_timeout(), std::exception);
  EXPECT_THROW(timers_manager->execute_ready_timers(), std::exception);
  EXPECT_THROW(timers_manager->execute_head_timer(), std::exception);

  timers_manager->stop();

  EXPECT_NO_THROW(timers_manager->get_head_timeout());
  EXPECT_NO_THROW(timers_manager->execute_ready_timers());
  EXPECT_NO_THROW(timers_manager->execute_head_timer());
}

TEST_F(TestTimersManager, add_timer_twice)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  auto t = TimerT::make_shared(1ms, CallbackT(), rclcpp::contexts::get_global_default_context());

  timers_manager->add_timer(t);
  EXPECT_NO_THROW(timers_manager->add_timer(t));
}

TEST_F(TestTimersManager, add_nullptr)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  EXPECT_THROW(timers_manager->add_timer(nullptr), std::exception);
}

TEST_F(TestTimersManager, head_not_ready)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  size_t t_runs = 0;
  auto t = TimerT::make_shared(
    10s,
    [&t_runs]() {
      t_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  timers_manager->add_timer(t);

  // Timer will take 10s to get ready, so nothing to execute here
  bool ret = timers_manager->execute_head_timer();
  EXPECT_FALSE(ret);
  EXPECT_EQ(0u, t_runs);
}

TEST_F(TestTimersManager, timers_order)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  size_t t1_runs = 0;
  auto t1 = TimerT::make_shared(
    1ms,
    [&t1_runs]() {
      t1_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  size_t t2_runs = 0;
  auto t2 = TimerT::make_shared(
    3ms,
    [&t2_runs]() {
      t2_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  size_t t3_runs = 0;
  auto t3 = TimerT::make_shared(
    10ms,
    [&t3_runs]() {
      t3_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  // Add timers in a random order
  timers_manager->add_timer(t2);
  timers_manager->add_timer(t3);
  timers_manager->add_timer(t1);

  std::this_thread::sleep_for(1ms);
  timers_manager->execute_ready_timers();
  EXPECT_EQ(1u, t1_runs);
  EXPECT_EQ(0u, t2_runs);
  EXPECT_EQ(0u, t3_runs);

  std::this_thread::sleep_for(3ms);
  timers_manager->execute_ready_timers();
  EXPECT_EQ(2u, t1_runs);
  EXPECT_EQ(1u, t2_runs);
  EXPECT_EQ(0u, t3_runs);

  std::this_thread::sleep_for(10ms);
  timers_manager->execute_ready_timers();
  EXPECT_EQ(3u, t1_runs);
  EXPECT_EQ(2u, t2_runs);
  EXPECT_EQ(1u, t3_runs);

  timers_manager->remove_timer(t1);

  std::this_thread::sleep_for(3ms);
  timers_manager->execute_ready_timers();
  EXPECT_EQ(3u, t1_runs);
  EXPECT_EQ(3u, t2_runs);
  EXPECT_EQ(1u, t3_runs);
}

TEST_F(TestTimersManager, start_stop_timers_thread)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  auto t = TimerT::make_shared(1ms, []() {}, rclcpp::contexts::get_global_default_context());
  timers_manager->add_timer(t);

  // Calling start multiple times will throw an error
  EXPECT_NO_THROW(timers_manager->start());
  EXPECT_THROW(timers_manager->start(), std::exception);

  // Calling stop multiple times does not throw an error
  EXPECT_NO_THROW(timers_manager->stop());
  EXPECT_NO_THROW(timers_manager->stop());
}

TEST_F(TestTimersManager, timers_thread)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  size_t t1_runs = 0;
  auto t1 = TimerT::make_shared(
    1ms,
    [&t1_runs]() {
      t1_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  size_t t2_runs = 0;
  auto t2 = TimerT::make_shared(
    1ms,
    [&t2_runs]() {
      t2_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  // Add timers
  timers_manager->add_timer(t1);
  timers_manager->add_timer(t2);

  // Run timers thread for a while
  timers_manager->start();
  std::this_thread::sleep_for(5ms);
  timers_manager->stop();

  EXPECT_LT(1u, t1_runs);
  EXPECT_LT(1u, t2_runs);
  EXPECT_EQ(t1_runs, t2_runs);
}

TEST_F(TestTimersManager, destructor)
{
  size_t t_runs = 0;
  auto t = TimerT::make_shared(
    1ms,
    [&t_runs]() {
      t_runs++;
    },
    rclcpp::contexts::get_global_default_context());
  std::weak_ptr<TimerT> t_weak = t;

  // When the timers manager is destroyed, it will stop the thread
  // and clear the timers
  {
    auto timers_manager = std::make_shared<TimersManager>(
      rclcpp::contexts::get_global_default_context());

    timers_manager->add_timer(std::move(t));

    timers_manager->start();
    std::this_thread::sleep_for(3ms);

    EXPECT_LT(1u, t_runs);
  }

  // The thread is not running anymore, so this value does not increase
  size_t runs = t_runs;
  std::this_thread::sleep_for(3ms);
  EXPECT_EQ(runs, t_runs);
  EXPECT_FALSE(t_weak.lock() != nullptr);
}

TEST_F(TestTimersManager, add_remove_while_thread_running)
{
  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  size_t t1_runs = 0;
  auto t1 = TimerT::make_shared(
    1ms,
    [&t1_runs]() {
      t1_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  size_t t2_runs = 0;
  auto t2 = TimerT::make_shared(
    1ms,
    [&t2_runs]() {
      t2_runs++;
    },
    rclcpp::contexts::get_global_default_context());

  // Add timers
  timers_manager->add_timer(t1);

  // Start timers thread
  timers_manager->start();

  // After a while remove t1 and add t2
  std::this_thread::sleep_for(5ms);
  timers_manager->remove_timer(t1);
  size_t tmp_t1 = t1_runs;
  timers_manager->add_timer(t2);

  // Wait some more time and then stop
  std::this_thread::sleep_for(5ms);
  timers_manager->stop();

  // t1 has stopped running
  EXPECT_EQ(tmp_t1, t1_runs);
  // t2 is correctly running
  EXPECT_LT(1u, t2_runs);
}

TEST_F(TestTimersManager, infinite_loop)
{
  // This test makes sure that even if timers have a period shorter than the duration
  // of their callback the functions never block indefinitely.

  auto timers_manager = std::make_shared<TimersManager>(
    rclcpp::contexts::get_global_default_context());

  size_t t1_runs = 0;
  auto t1 = TimerT::make_shared(
    1ms,
    [&t1_runs]() {
      t1_runs++;
      std::this_thread::sleep_for(5ms);
    },
    rclcpp::contexts::get_global_default_context());

  size_t t2_runs = 0;
  auto t2 = TimerT::make_shared(
    1ms,
    [&t2_runs]() {
      t2_runs++;
      std::this_thread::sleep_for(5ms);
    },
    rclcpp::contexts::get_global_default_context());

  timers_manager->add_timer(t1);
  timers_manager->add_timer(t2);

  // Sleep for enough time to trigger timers
  std::this_thread::sleep_for(3ms);
  timers_manager->execute_ready_timers();
  EXPECT_EQ(1u, t1_runs);
  EXPECT_EQ(1u, t2_runs);

  // Due to the long execution of timer callbacks, timers are already ready
  bool ret = timers_manager->execute_head_timer();
  EXPECT_TRUE(ret);
  EXPECT_EQ(3u, t1_runs + t2_runs);

  // Start a timers thread
  timers_manager->start();
  std::this_thread::sleep_for(10ms);
  timers_manager->stop();

  EXPECT_LT(3u, t1_runs + t2_runs);
  EXPECT_LT(1u, t1_runs);
  EXPECT_LT(1u, t2_runs);
}
