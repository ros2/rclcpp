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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class TestEventsCBGExecutorReentrant : public testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}
};

/*
 * Test that multiple callbacks from the same reentrant callback group can
 * be executed at the same time.
 *
 * The test creates two subscribers in a single reentrant callback group that
 * listen to the same topic. Whichever subscriber executes first waits for
 * the other to also start executing. If this waiting results in a timeout,
 * then we know that the second subscriber wasn't able to execute because the
 * executor handled the callback group incorrectly.
 *
 * Related issue: https://github.com/ros2/rclcpp/issues/3175
 */
TEST_F(TestEventsCBGExecutorReentrant, reentract_callback_group_runs_concurrently)
{
  auto node = std::make_shared<rclcpp::Node>("test_events_cbg_executor_reentrant");

  std::mutex rendezvous_mutex;
  std::condition_variable rendezvous_cv;
  auto rendezvous = [&](bool & own_started, const bool & other_started) {
      std::unique_lock<std::mutex> lock(rendezvous_mutex);
      own_started = true;
      rendezvous_cv.notify_all();
      return rendezvous_cv.wait_for(lock, 2s, [&other_started]() {return other_started;});
    };

  rclcpp::SubscriptionOptions sub_opt;
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sub_opt.callback_group = cbg;

  bool sub1_started = false;
  bool sub2_started = false;
  std::atomic_bool sub1_finished{false};
  std::atomic_bool sub2_finished{false};
  std::atomic_bool sub1_timed_out{false};
  std::atomic_bool sub2_timed_out{false};

  auto sub_1 = node->create_subscription<test_msgs::msg::Empty>(
    "empty", 10,
    [&](test_msgs::msg::Empty) {
      if (!rendezvous(sub1_started, sub2_started)) {
        sub1_timed_out = true;
      }
      sub1_finished = true;
    },
    sub_opt);

  auto sub_2 = node->create_subscription<test_msgs::msg::Empty>(
    "empty", 10,
    [&](test_msgs::msg::Empty) {
      if (!rendezvous(sub2_started, sub1_started)) {
        sub2_timed_out = true;
      }
      sub2_finished = true;
    },
    sub_opt);

  auto pub = node->create_publisher<test_msgs::msg::Empty>("empty", 10);

  auto executor = rclcpp::executors::EventsCBGExecutor(rclcpp::ExecutorOptions(), 2u);
  ASSERT_GT(executor.get_number_of_threads(), 1u);

  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  /*
   * Publish (and re-publish, in case discovery hasn't completed yet) until
   * both callbacks have run to completion
   */
  test_msgs::msg::Empty msg;
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
    !(sub1_finished.load() && sub2_finished.load()))
  {
    pub->publish(msg);
    std::this_thread::sleep_for(500ms);
  }

  executor.cancel();
  spin_thread.join();

  EXPECT_TRUE(sub1_finished.load()) << "sub 1 never ran";
  EXPECT_TRUE(sub2_finished.load()) << "sub 2 never ran";
  EXPECT_FALSE(sub1_timed_out.load()) << "sub 1 timed out waiting for sub 2 to start running -- "
                                         "the reentrant callback group appears to be serialized";
  EXPECT_FALSE(sub2_timed_out.load()) << "sub 2 timed out waiting for sub 1 to start running -- "
                                         "the reentrant callback group appears to be serialized";
}

/*
 * Test that an entity of a reentrant callback group which becomes ready while
 * another entity of the same group is already executing gets executed
 * concurrently, instead of waiting for the executing one to finish.
 *
 * The second message is only published once the first callback is running,
 * so the event for the second subscription always arrives while the group
 * is busy.
 *
 * Related issue: https://github.com/ros2/rclcpp/issues/3328
 */
TEST_F(TestEventsCBGExecutorReentrant, entity_ready_during_execution_runs_concurrently)
{
  auto node = std::make_shared<rclcpp::Node>("test_events_cbg_executor_reentrant_busy");

  std::mutex mutex;
  std::condition_variable cv;
  bool sub_a_started = false;
  bool sub_b_started = false;
  std::atomic_bool sub_a_timed_out{false};

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sub_a = node->create_subscription<test_msgs::msg::Empty>(
    "reentrant_busy_a", 10,
    [&](test_msgs::msg::Empty) {
      std::unique_lock<std::mutex> lock(mutex);
      sub_a_started = true;
      cv.notify_all();
      if (!cv.wait_for(lock, 5s, [&]() {return sub_b_started;})) {
        sub_a_timed_out = true;
      }
    },
    sub_opt);

  auto sub_b = node->create_subscription<test_msgs::msg::Empty>(
    "reentrant_busy_b", 10,
    [&](test_msgs::msg::Empty) {
      std::lock_guard<std::mutex> lock(mutex);
      sub_b_started = true;
      cv.notify_all();
    },
    sub_opt);

  auto pub_a = node->create_publisher<test_msgs::msg::Empty>("reentrant_busy_a", 10);
  auto pub_b = node->create_publisher<test_msgs::msg::Empty>("reentrant_busy_b", 10);

  auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((pub_a->get_subscription_count() == 0 || pub_b->get_subscription_count() == 0) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(pub_a->get_subscription_count(), 1u);
  ASSERT_EQ(pub_b->get_subscription_count(), 1u);

  rclcpp::executors::EventsCBGExecutor executor(rclcpp::ExecutorOptions(), 2u);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  pub_a->publish(test_msgs::msg::Empty());
  bool a_started;
  {
    std::unique_lock<std::mutex> lock(mutex);
    a_started = cv.wait_for(lock, 5s, [&]() {return sub_a_started;});
  }
  if (a_started) {
    pub_b->publish(test_msgs::msg::Empty());
  }

  bool b_started;
  {
    std::unique_lock<std::mutex> lock(mutex);
    b_started = cv.wait_for(lock, 10s, [&]() {return sub_b_started;});
  }

  executor.cancel();
  spin_thread.join();

  ASSERT_TRUE(a_started) << "sub a never ran";
  EXPECT_TRUE(b_started) << "sub b never ran";
  EXPECT_FALSE(sub_a_timed_out.load()) <<
    "sub b only ran after sub a finished -- the reentrant callback group appears to be serialized";
}
