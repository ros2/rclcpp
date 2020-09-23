// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "rclcpp/rate.hpp"

/*
   Basic tests for the Rate and WallRate classes.
 */
TEST(TestRate, rate_basics) {
  auto period = std::chrono::milliseconds(1000);
  auto offset = std::chrono::milliseconds(500);
  auto epsilon = std::chrono::milliseconds(100);
  double overrun_ratio = 1.5;

  auto start = std::chrono::system_clock::now();
  rclcpp::Rate r(period);
  EXPECT_EQ(period, r.period());
  ASSERT_FALSE(r.is_steady());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::system_clock::now();
  auto delta = one - start;
  EXPECT_LT(period, delta + epsilon);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::system_clock::now();
  delta = two - start;
  EXPECT_LT(2 * period, delta);
  EXPECT_GT(2 * period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::system_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::system_clock::now();
  delta = three - two_offset;
  EXPECT_LT(period, delta + epsilon);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset + period);
  auto four = std::chrono::system_clock::now();
  ASSERT_FALSE(r.sleep());
  auto five = std::chrono::system_clock::now();
  delta = five - four;
  ASSERT_TRUE(epsilon > delta);
}

TEST(TestRate, wall_rate_basics) {
  auto period = std::chrono::milliseconds(100);
  auto offset = std::chrono::milliseconds(50);
  auto epsilon = std::chrono::milliseconds(1);
  double overrun_ratio = 1.5;

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate r(period);
  EXPECT_EQ(period, r.period());
  ASSERT_TRUE(r.is_steady());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::steady_clock::now();
  auto delta = one - start;
  EXPECT_LT(period, delta);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::steady_clock::now();
  delta = two - start;
  EXPECT_LT(2 * period, delta + epsilon);
  EXPECT_GT(2 * period * overrun_ratio, delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::steady_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::steady_clock::now();
  delta = three - two_offset;
  EXPECT_LT(period, delta);
  EXPECT_GT(period * overrun_ratio, delta);

  rclcpp::sleep_for(offset + period);
  auto four = std::chrono::steady_clock::now();
  ASSERT_FALSE(r.sleep());
  auto five = std::chrono::steady_clock::now();
  delta = five - four;
  EXPECT_GT(epsilon, delta);
}

TEST(TestRate, from_double) {
  {
    rclcpp::WallRate rate(1.0);
    EXPECT_EQ(std::chrono::seconds(1), rate.period());
  }
  {
    rclcpp::WallRate rate(2.0);
    EXPECT_EQ(std::chrono::milliseconds(500), rate.period());
  }
  {
    rclcpp::WallRate rate(0.5);
    EXPECT_EQ(std::chrono::seconds(2), rate.period());
  }
  {
    rclcpp::WallRate rate(4.0);
    EXPECT_EQ(std::chrono::milliseconds(250), rate.period());
  }
}
