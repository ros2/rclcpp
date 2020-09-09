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

#include <rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <test_msgs/action/fibonacci.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "action_msgs/msg/goal_info.h"
#include "rclcpp_action/server_goal_handle.hpp"
#include "./mocking_utils/patch.hpp"

class FibonacciServerGoalHandle
  : public rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>
{
public:
  FibonacciServerGoalHandle(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle,
    rclcpp_action::GoalUUID uuid,
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal> goal,
    std::function<void(
      const rclcpp_action::GoalUUID &, std::shared_ptr<void>)> on_terminal_state,
    std::function<void(const rclcpp_action::GoalUUID &)> on_executing,
    std::function<void(
      std::shared_ptr<test_msgs::action::Fibonacci::Impl::FeedbackMessage>)> publish_feedback)
  : rclcpp_action::ServerGoalHandle<test_msgs::action::Fibonacci>(
      rcl_handle, uuid, goal, on_terminal_state, on_executing, publish_feedback) {}

  bool try_cancel() {return try_canceling();}

  void cancel_goal() {_cancel_goal();}
};

class TestServerGoalHandle : public ::testing::Test
{
public:
  TestServerGoalHandle()
  : handle_(nullptr) {}

  void SetUp()
  {
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle =
      std::make_shared<rcl_action_goal_handle_t>();
    *rcl_handle.get() = rcl_action_get_zero_initialized_goal_handle();
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
    ASSERT_EQ(RCL_RET_OK, rcl_action_goal_handle_init(rcl_handle.get(), &goal_info, allocator));
    rclcpp_action::GoalUUID uuid;
    std::shared_ptr<const test_msgs::action::Fibonacci::Goal> goal =
      std::make_shared<const test_msgs::action::Fibonacci::Goal>();
    auto on_terminal_state = [](const rclcpp_action::GoalUUID &, std::shared_ptr<void>) {};
    auto on_executing = [](const rclcpp_action::GoalUUID &) {};
    auto publish_feedback =
      [](std::shared_ptr<test_msgs::action::Fibonacci::Impl::FeedbackMessage>) {};
    handle_ = std::make_unique<FibonacciServerGoalHandle>(
      rcl_handle, uuid, goal, on_terminal_state, on_executing, publish_feedback);
  }

protected:
  std::unique_ptr<FibonacciServerGoalHandle> handle_;
};

TEST_F(TestServerGoalHandle, construct_destruct) {
  EXPECT_FALSE(handle_->is_canceling());
  EXPECT_TRUE(handle_->is_active());
  EXPECT_FALSE(handle_->is_executing());
}

TEST_F(TestServerGoalHandle, cancel) {
  handle_->execute();
  EXPECT_TRUE(handle_->try_cancel());
  EXPECT_FALSE(handle_->is_canceling());
  EXPECT_FALSE(handle_->is_active());
  EXPECT_FALSE(handle_->is_executing());

  {
    auto mock_get_status = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_goal_handle_get_status, RCL_RET_ERROR);
    EXPECT_FALSE(handle_->try_cancel());
  }

  {
    auto mock_update_status = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_update_goal_state, RCL_RET_ERROR);
    auto mock_is_cancelable = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_goal_handle_is_cancelable, true);
    EXPECT_FALSE(handle_->try_cancel());
    EXPECT_THROW(handle_->cancel_goal(), rclcpp::exceptions::RCLError);

    test_msgs::action::Fibonacci::Result::SharedPtr result =
      std::make_shared<test_msgs::action::Fibonacci::Result>();
    EXPECT_THROW(handle_->canceled(result), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestServerGoalHandle, abort) {
  handle_->execute();
  test_msgs::action::Fibonacci::Result::SharedPtr result =
    std::make_shared<test_msgs::action::Fibonacci::Result>();
  handle_->abort(result);
  EXPECT_FALSE(handle_->is_canceling());
  EXPECT_FALSE(handle_->is_active());
  EXPECT_FALSE(handle_->is_executing());

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_update_goal_state, RCL_RET_ERROR);
  EXPECT_THROW(handle_->abort(result), rclcpp::exceptions::RCLError);
}

TEST_F(TestServerGoalHandle, succeed) {
  handle_->execute();
  test_msgs::action::Fibonacci::Result::SharedPtr result =
    std::make_shared<test_msgs::action::Fibonacci::Result>();
  handle_->succeed(result);
  EXPECT_FALSE(handle_->is_canceling());
  EXPECT_FALSE(handle_->is_active());
  EXPECT_FALSE(handle_->is_executing());

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_update_goal_state, RCL_RET_ERROR);
  EXPECT_THROW(handle_->succeed(result), rclcpp::exceptions::RCLError);
}


TEST_F(TestServerGoalHandle, execute) {
  handle_->execute();
  EXPECT_FALSE(handle_->is_canceling());
  EXPECT_TRUE(handle_->is_active());
  EXPECT_TRUE(handle_->is_executing());

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_update_goal_state, RCL_RET_ERROR);
  EXPECT_THROW(handle_->execute(), rclcpp::exceptions::RCLError);
}

TEST_F(TestServerGoalHandle, rcl_action_goal_handle_get_status_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp_action", rcl_action_goal_handle_get_status, RCL_RET_ERROR);

  EXPECT_THROW(handle_->is_canceling(), rclcpp::exceptions::RCLError);
  EXPECT_NO_THROW(handle_->is_active());
  EXPECT_THROW(handle_->is_executing(), rclcpp::exceptions::RCLError);
}
