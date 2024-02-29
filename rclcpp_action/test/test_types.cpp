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
  rclcpp_action::convert(goal_info, &goal_id);
  for (uint8_t i = 0; i < UUID_SIZE; ++i) {
    EXPECT_EQ(goal_info.goal_id.uuid[i], goal_id[i]);
  }
}

TEST(TestActionTypes, goal_uuid_hashing_algorithm_collisions) {
  // Use predefined IDs that have been known to fail with previous hashing algorithms
  rclcpp_action::GoalUUID g1_1 = {0x0b,0x76,0xbe,0x24,0x5a,0x94,0x42,0xc2,0xfe,0xa2,0x20,0x96,0xa7,0x43,0x4a,0xa7};
  rclcpp_action::GoalUUID g1_2 = {0x04,0x79,0x41,0xdf,0x7d,0x4d,0xb5,0xe2,0xeb,0x9b,0xeb,0x59,0xe2,0x81,0x26,0x49};

  rclcpp_action::GoalUUID g2_1 = {0x17,0xe6,0xf4,0xff,0x9d,0xb6,0xaa,0x53,0x59,0x84,0x82,0xf0,0x1d,0x46,0xe0,0xcf};
  rclcpp_action::GoalUUID g2_2 = {0x0c,0xbb,0x81,0x55,0xe1,0x04,0x7b,0x61,0x76,0xf6,0x51,0x96,0xbe,0x83,0xb4,0xa1};

  rclcpp_action::GoalUUID g3_1 = {0x00,0x20,0x83,0x8d,0x73,0x49,0x4e,0x9c,0x8d,0x51,0x8e,0x34,0xaf,0xb4,0x75,0xd1};
  rclcpp_action::GoalUUID g3_2 = {0x0d,0xfb,0x53,0x2f,0x13,0x89,0xa9,0xf3,0x4a,0x36,0xa4,0x84,0x77,0xc9,0xf7,0x40};

  rclcpp_action::GoalUUID g4_1 = {0x08,0x6c,0x88,0x06,0xf6,0x8f,0xa5,0x9c,0xae,0xb8,0xa3,0x22,0xf7,0xba,0x9b,0x3c};
  rclcpp_action::GoalUUID g4_2 = {0x0b,0xd7,0x63,0x72,0x16,0xc9,0x72,0x7b,0x06,0x59,0x59,0xdd,0xb8,0xf4,0xb5,0xee};

  size_t hashed_g1_1 = std::hash<rclcpp_action::GoalUUID>()(g1_1);
  size_t hashed_g1_2 = std::hash<rclcpp_action::GoalUUID>()(g1_2);
  EXPECT_NE(hashed_g1_1, hashed_g1_2);

  size_t hashed_g2_1 = std::hash<rclcpp_action::GoalUUID>()(g2_1);
  size_t hashed_g2_2 = std::hash<rclcpp_action::GoalUUID>()(g2_2);
  EXPECT_NE(hashed_g2_1, hashed_g2_2);

  size_t hashed_g3_1 = std::hash<rclcpp_action::GoalUUID>()(g3_1);
  size_t hashed_g3_2 = std::hash<rclcpp_action::GoalUUID>()(g3_2);
  EXPECT_NE(hashed_g3_1, hashed_g3_2);

  size_t hashed_g4_1 = std::hash<rclcpp_action::GoalUUID>()(g4_1);
  size_t hashed_g4_2 = std::hash<rclcpp_action::GoalUUID>()(g4_2);
  EXPECT_NE(hashed_g4_1, hashed_g4_2);
}

TEST(TestActionTypes, goal_uuid_to_hashed_uuid_random) {
  // Use std::random_device to seed the generator of goal IDs.
  std::random_device rd;
  std::independent_bits_engine<std::default_random_engine, 8, decltype(rd())> random_bytes_generator(rd());
  std::vector<size_t> hashed_guuids;
  constexpr size_t iterations = 1000;

  for (size_t i = 0; i < iterations; i++) {
    rclcpp_action::GoalUUID goal_id;

    // Generate random bytes for each element of the array
    for (auto& element : goal_id) {
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
