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

#include <chrono>
#include <gtest/gtest.h>

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

using namespace std::chrono_literals;

enum class TimerType
{
  WALL_TIMER,
  GENERIC_TIMER,
};

class TestLifecycleTimer : public ::testing::TestWithParam<TimerType> {
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

TEST_P(TestLifecycleTimer, timer_becomes_activated_and_deactivated_with_node) {
  bool is_executed = false;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node");

  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
  auto error = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::ERROR;
  auto ret = error;

  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(success, ret) << "node transition failed";
  ret = error;

  TimerType timer_type = GetParam();
  std::function<void()> callback = [&]() {is_executed = true;};
  rclcpp_lifecycle::LifecycleGenericTimer<std::function<void()>>::SharedPtr
    timer;
  switch (timer_type) {
    case TimerType::WALL_TIMER: {
        timer = node->create_lifecycle_wall_timer(100ms, callback);
        break;
      }
    case TimerType::GENERIC_TIMER: {
        timer = node->create_lifecycle_timer(100ms, callback);
        break;
      }
  }

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());

  auto spinner = std::thread([&]() {exec->spin();});

  // Timer should not be executed until the node is activated
  ASSERT_FALSE(timer->is_activated())
      << "managed timer is active while its node is not";
  std::this_thread::sleep_for(500ms);
  ASSERT_FALSE(is_executed)
      << "managed timer was executed while node was inactive";

  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE), ret);
  ASSERT_EQ(success, ret) << "node transition failed";
  ret = error;

  // Now, the timer should be activated and its callback should be executed
  ASSERT_TRUE(timer->is_activated())
      << "managed timer should be activated with the node";
  std::this_thread::sleep_for(500ms);
  ASSERT_TRUE(is_executed)
      << "managed timer was not executed while the node was active";

  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret) << "node transition failed";
  ret = error;
  is_executed = false;

  // Timer should not be executed after deactivation
  ASSERT_FALSE(timer->is_activated())
      << "managed timer is active while its node is not";
  std::this_thread::sleep_for(500ms);
  ASSERT_FALSE(is_executed)
      << "managed timer was executed while node was inactive";

  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_INACTIVE_SHUTDOWN),
      ret);
  ASSERT_EQ(success, ret) << "node transition failed";
  ret = error;

  exec->cancel();
  spinner.join();
}

INSTANTIATE_TEST_SUITE_P(
    PerTimerType, TestLifecycleTimer,
    ::testing::Values(TimerType::WALL_TIMER, TimerType::GENERIC_TIMER),
  [](const ::testing::TestParamInfo<TimerType> & info) -> std::string {
    switch (info.param) {
      case TimerType::WALL_TIMER:
        return std::string("wall_timer");
      case TimerType::GENERIC_TIMER:
        return std::string("generic_timer");
      default:
        break;
    }
    return std::string("unknown");
    });
