// Copyright 2024 Open Source Robotics Foundation, Inc.
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
 * This test checks that when callback groups go out of scope, that their associated executable
 * entities should not be returned as valid executables.
 *
 * The test makes use of a bit of executor internals, but is meant to prevent regressions of behavior.
 * Ref: https://github.com/ros2/rclcpp/issues/2474
 */

#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>

std::chrono::milliseconds g_timer_period {1};

class CustomExecutor : public rclcpp::Executor
{
public:
  explicit CustomExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions())
  : rclcpp::Executor(options)
  {}

  ~CustomExecutor() override = default;

  void spin() override {}

  void collect()
  {
    this->collect_entities();
  }

  void wait()
  {
    this->wait_for_work(g_timer_period * 10);
  }

  size_t collected_timers() const
  {
    return this->current_collection_.timers.size();
  }

  rclcpp::AnyExecutable next()
  {
    rclcpp::AnyExecutable ret;
    this->get_next_ready_executable(ret);
    return ret;
  }
};


TEST(TestCallbackGroup, valid_callback_group)
{
  rclcpp::init(0, nullptr);

  // Create a timer associated with a callback group
  auto node = std::make_shared<rclcpp::Node>("node");

  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  auto timer_callback = [&promise]() {
      promise.set_value();
    };

  // Add the callback group to the executor
  auto executor = CustomExecutor();
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  auto timer = node->create_wall_timer(g_timer_period, timer_callback, cbg);
  executor.add_callback_group(cbg, node->get_node_base_interface());

  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(future, std::chrono::seconds(10)));

  // Collect the entities
  executor.collect();
  EXPECT_EQ(1u, executor.collected_timers());

  executor.wait();
  auto next_executable = executor.next();
  EXPECT_EQ(timer, next_executable.timer);
  EXPECT_EQ(cbg, next_executable.callback_group);
  EXPECT_NE(nullptr, next_executable.data);

  EXPECT_EQ(nullptr, next_executable.client);
  EXPECT_EQ(nullptr, next_executable.service);
  EXPECT_EQ(nullptr, next_executable.subscription);
  EXPECT_EQ(nullptr, next_executable.waitable);

  rclcpp::shutdown();
}

TEST(TestCallbackGroup, invalid_callback_group)
{
  rclcpp::init(0, nullptr);

  // Create a timer associated with a callback group
  auto node = std::make_shared<rclcpp::Node>("node");

  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  auto timer_callback = [&promise]() {
      promise.set_value();
    };

  // Add the callback group to the executor
  auto executor = CustomExecutor();
  auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto timer = node->create_wall_timer(g_timer_period, timer_callback, cbg);
  executor.add_callback_group(cbg, node->get_node_base_interface());

  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(future, std::chrono::seconds(10)));

  // Collect the entities
  executor.collect();
  EXPECT_EQ(1u, executor.collected_timers());

  executor.wait();

  cbg.reset();

  // Since the callback group has been reset, this should not be allowed to
  // be a valid executable (timer and cbg should both be nullptr).
  // In the regression, timer == next_executable.timer whil
  // next_executable.callback_group == nullptr, which was incorrect.
  auto next_executable = executor.next();
  EXPECT_EQ(nullptr, next_executable.timer);
  EXPECT_EQ(nullptr, next_executable.callback_group);

  EXPECT_EQ(nullptr, next_executable.client);
  EXPECT_EQ(nullptr, next_executable.service);
  EXPECT_EQ(nullptr, next_executable.subscription);
  EXPECT_EQ(nullptr, next_executable.waitable);
  EXPECT_EQ(nullptr, next_executable.data);

  rclcpp::shutdown();
}
