// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <test_msgs/action/fibonacci.hpp>

using rosidl_generator_traits::is_message;
using rosidl_generator_traits::is_service;
using rosidl_generator_traits::is_action;
using rosidl_generator_traits::is_action_goal;
using rosidl_generator_traits::is_action_result;
using rosidl_generator_traits::is_action_feedback;

TEST(TestActionTraits, is_action) {
  using Fibonacci = test_msgs::action::Fibonacci;

  // Top level definition is an action
  EXPECT_FALSE(is_message<Fibonacci>());
  EXPECT_FALSE(is_service<Fibonacci>());
  EXPECT_TRUE(is_action<Fibonacci>());
  EXPECT_FALSE(is_action_goal<Fibonacci>());
  EXPECT_FALSE(is_action_result<Fibonacci>());
  EXPECT_FALSE(is_action_feedback<Fibonacci>());

  // Goal is an action_goal as well as a message
  EXPECT_TRUE(is_message<Fibonacci::Goal>());
  EXPECT_FALSE(is_service<Fibonacci::Goal>());
  EXPECT_FALSE(is_action<Fibonacci::Goal>());
  EXPECT_TRUE(is_action_goal<Fibonacci::Goal>());
  EXPECT_FALSE(is_action_result<Fibonacci::Goal>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Goal>());

  // Result is an action_result as well as a message
  EXPECT_TRUE(is_message<Fibonacci::Result>());
  EXPECT_FALSE(is_service<Fibonacci::Result>());
  EXPECT_FALSE(is_action<Fibonacci::Result>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Result>());
  EXPECT_TRUE(is_action_result<Fibonacci::Result>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Result>());

  // Feedback is an action_feedback as well as a message
  EXPECT_TRUE(is_message<Fibonacci::Feedback>());
  EXPECT_FALSE(is_service<Fibonacci::Feedback>());
  EXPECT_FALSE(is_action<Fibonacci::Feedback>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Feedback>());
  EXPECT_FALSE(is_action_result<Fibonacci::Feedback>());
  EXPECT_TRUE(is_action_feedback<Fibonacci::Feedback>());
}

TEST(TestActionTraits, is_action_impl) {
  using Fibonacci = test_msgs::action::Fibonacci;

  // Test traits on some of the internal implementation of actions
  EXPECT_FALSE(is_message<Fibonacci::Impl::SendGoalService>());
  EXPECT_TRUE(is_service<Fibonacci::Impl::SendGoalService>());
  EXPECT_FALSE(is_action<Fibonacci::Impl::SendGoalService>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Impl::SendGoalService>());
  EXPECT_FALSE(is_action_result<Fibonacci::Impl::SendGoalService>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Impl::SendGoalService>());

  EXPECT_FALSE(is_message<Fibonacci::Impl::GetResultService>());
  EXPECT_TRUE(is_service<Fibonacci::Impl::GetResultService>());
  EXPECT_FALSE(is_action<Fibonacci::Impl::GetResultService>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Impl::GetResultService>());
  EXPECT_FALSE(is_action_result<Fibonacci::Impl::GetResultService>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Impl::GetResultService>());

  EXPECT_FALSE(is_message<Fibonacci::Impl::CancelGoalService>());
  EXPECT_TRUE(is_service<Fibonacci::Impl::CancelGoalService>());
  EXPECT_FALSE(is_action<Fibonacci::Impl::CancelGoalService>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Impl::CancelGoalService>());
  EXPECT_FALSE(is_action_result<Fibonacci::Impl::CancelGoalService>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Impl::CancelGoalService>());

  EXPECT_TRUE(is_message<Fibonacci::Impl::FeedbackMessage>());
  EXPECT_FALSE(is_service<Fibonacci::Impl::FeedbackMessage>());
  EXPECT_FALSE(is_action<Fibonacci::Impl::FeedbackMessage>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Impl::FeedbackMessage>());
  EXPECT_FALSE(is_action_result<Fibonacci::Impl::FeedbackMessage>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Impl::FeedbackMessage>());

  EXPECT_TRUE(is_message<Fibonacci::Impl::GoalStatusMessage>());
  EXPECT_FALSE(is_service<Fibonacci::Impl::GoalStatusMessage>());
  EXPECT_FALSE(is_action<Fibonacci::Impl::GoalStatusMessage>());
  EXPECT_FALSE(is_action_goal<Fibonacci::Impl::GoalStatusMessage>());
  EXPECT_FALSE(is_action_result<Fibonacci::Impl::GoalStatusMessage>());
  EXPECT_FALSE(is_action_feedback<Fibonacci::Impl::GoalStatusMessage>());
}
