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
#include <random>
#include "rclcpp_action/types.hpp"

TEST(TestActionTypes, goal_uuid_to_string) {
  rclcpp_action::GoalUUID goal_id;
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = i;
  }
  EXPECT_STREQ("00010203-0405-0607-0809-0a0b0c0d0e0f", rclcpp_action::to_string(goal_id).c_str());

  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = static_cast<uint8_t>(16u + i);
  }
  EXPECT_STREQ("10111213-1415-1617-1819-1a1b1c1d1e1f", rclcpp_action::to_string(goal_id).c_str());

  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = static_cast<uint8_t>(std::numeric_limits<uint8_t>::max() - i);
  }
  EXPECT_STREQ("fffefdfc-fbfa-f9f8-f7f6-f5f4f3f2f1f0", rclcpp_action::to_string(goal_id).c_str());
}

TEST(TestActionTypes, goal_uuid_to_rcl_action_goal_info) {
  rclcpp_action::GoalUUID goal_id;
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    goal_id[i] = i;
  }
  ASSERT_THROW(rclcpp_action::convert(goal_id, nullptr), std::invalid_argument);
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

  ASSERT_THROW(rclcpp_action::convert(goal_info, nullptr), std::invalid_argument);
  rclcpp_action::GoalUUID goal_id;
  rclcpp_action::convert(goal_info, &goal_id);
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    EXPECT_EQ(goal_info.goal_id.uuid[i], goal_id[i]);
  }
}

TEST(TestActionTypes, goal_uuid_to_hashed_uuid_random) {
  // Use std::random_device to seed the generator of goal IDs.
  std::random_device rd;
  std::independent_bits_engine<
    std::default_random_engine, 8, decltype(rd())> random_bytes_generator(rd());

  std::vector<size_t> hashed_guuids;
  constexpr size_t iterations = 1000;

  for (size_t i = 0; i < iterations; i++) {
    rclcpp_action::GoalUUID goal_id;

    // Generate random bytes for each element of the array
    for (auto & element : goal_id) {
      element = static_cast<uint8_t>(random_bytes_generator());
    }

    size_t new_hashed_guuid = std::hash<rclcpp_action::GoalUUID>()(goal_id);

    // Search for any prevoius hashed goal_id with the same value
    for (auto prev_hashed_guuid : hashed_guuids) {
      EXPECT_NE(prev_hashed_guuid, new_hashed_guuid);
      if (prev_hashed_guuid == new_hashed_guuid) {
        // Fail before the first occurrence of a collision
        GTEST_FAIL();
      }
    }

    hashed_guuids.push_back(new_hashed_guuid);
  }
}
