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

#include <limits>
#include "rclcpp_action/types.hpp"

TEST(TestActionTypes, goal_uuid_to_string) {
  rclcpp_action::GoalUUID goal_id;
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = i;
  }
  EXPECT_STREQ("0123456789abcdef", rclcpp_action::to_string(goal_id).c_str());

  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = static_cast<uint8_t>(16u + i);
  }
  EXPECT_STREQ("101112131415161718191a1b1c1d1e1f", rclcpp_action::to_string(goal_id).c_str());

  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = static_cast<uint8_t>(std::numeric_limits<uint8_t>::max() - i);
  }
  EXPECT_STREQ("fffefdfcfbfaf9f8f7f6f5f4f3f2f1f0", rclcpp_action::to_string(goal_id).c_str());
}

TEST(TestActionTypes, goal_uuid_to_rcl_action_goal_info) {
  rclcpp_action::GoalUUID goal_id;
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = i;
  }
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rclcpp_action::convert(goal_id, &goal_info);
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    EXPECT_EQ(goal_info.goal_id.uuid[i], goal_id[i]);
  }
}

TEST(TestActionTypes, rcl_action_goal_info_to_goal_uuid) {
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_info.goal_id.uuid[i] = i;
  }

  rclcpp_action::GoalUUID goal_id;
  rclcpp_action::convert(goal_id, &goal_info);
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    EXPECT_EQ(goal_info.goal_id.uuid[i], goal_id[i]);
  }
}
