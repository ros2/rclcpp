// Copyright 2023 Elroy Air, Inc.
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
#include <memory>
#include <mutex>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_timer.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using namespace std::chrono_literals;


class TestLifecycleTimer : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_timer_node");
    timer_called_ = false;
    wall_timer_called_ = false;
    timer_ = node_->create_lifecycle_timer(
      10ms, std::bind(&TestLifecycleTimer::timer_callback, this));
    wall_timer_ = node_->create_lifecycle_wall_timer(
      10ms, std::bind(&TestLifecycleTimer::wall_timer_callback, this));

    spinner_ = std::thread(&TestLifecycleTimer::spin, this);
  }

  void TearDown()
  {
    {
      std::lock_guard<std::mutex> guard(shutdown_mutex_);
      rclcpp::shutdown();
    }
    spinner_.join();
  }

protected:
  void spin()
  {
    while (true) {
      {
        std::lock_guard<std::mutex> guard(shutdown_mutex_);
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::spin_some(node_->get_node_base_interface());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void timer_callback()
  {
    timer_called_ = true;
  }

  void wall_timer_callback()
  {
    wall_timer_called_ = true;
  }

  using CallbackT = std::shared_ptr<rclcpp_lifecycle::LifecycleTimer
      <std::_Bind<void(TestLifecycleTimer::* (TestLifecycleTimer *))()>,
      (void *) nullptr>>; // NOLINT
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  CallbackT timer_;
  CallbackT wall_timer_;
  std::atomic<bool> timer_called_;
  std::atomic<bool> wall_timer_called_;

  std::mutex shutdown_mutex_;
  std::thread spinner_;
};

TEST_F(TestLifecycleTimer, timer_managed_by_node) {
  // Test that the timers transition with the node.
  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto reset_key = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  auto ret = reset_key;

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, node_->get_current_state().id());
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  EXPECT_TRUE(timer_->is_activated());
  EXPECT_TRUE(wall_timer_->is_activated());

  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  (void)ret;  // Make clang happy
  EXPECT_FALSE(timer_->is_activated());
  EXPECT_FALSE(wall_timer_->is_activated());
}

TEST_F(TestLifecycleTimer, timer_callback_only_when_active) {
  // Test that the callback only gets called when the timer is active.
  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto reset_key = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  auto ret = reset_key;

  // Timer should not have been called yet
  EXPECT_EQ(timer_called_, false);
  EXPECT_EQ(wall_timer_called_, false);
  // Transition to configure
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, node_->get_current_state().id());
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Timer should not have been called yet.
  EXPECT_EQ(timer_called_, false);
  EXPECT_EQ(wall_timer_called_, false);

  // Transition to activate
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Timer should have been called.
  EXPECT_EQ(timer_called_, true);
  EXPECT_EQ(wall_timer_called_, true);

  // Transition to deactivate
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  timer_called_ = false;
  wall_timer_called_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Timer should not have been called.
  EXPECT_EQ(timer_called_, false);
  EXPECT_EQ(wall_timer_called_, false);
}
