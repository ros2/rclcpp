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
  auto period = std::chrono::milliseconds(100);
  auto offset = std::chrono::milliseconds(50);
  auto epsilon = std::chrono::milliseconds(1);
  double overrun_ratio = 1.5;

  auto start = std::chrono::system_clock::now();
  rclcpp::Rate r(period);
  ASSERT_FALSE(r.is_steady());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::system_clock::now();
  auto delta = one - start;
  ASSERT_TRUE(period < delta);
  ASSERT_TRUE(period * overrun_ratio > delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::system_clock::now();
  delta = two - start;
  ASSERT_TRUE(2 * period < delta);
  ASSERT_TRUE(2 * period * overrun_ratio > delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::system_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::system_clock::now();
  delta = three - two_offset;
  ASSERT_TRUE(period < delta);
  ASSERT_TRUE(period * overrun_ratio > delta);

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
  ASSERT_TRUE(r.is_steady());
  ASSERT_TRUE(r.sleep());
  auto one = std::chrono::steady_clock::now();
  auto delta = one - start;
  ASSERT_TRUE(period < delta);
  ASSERT_TRUE(period * overrun_ratio > delta);

  rclcpp::sleep_for(offset);
  ASSERT_TRUE(r.sleep());
  auto two = std::chrono::steady_clock::now();
  delta = two - start;
  ASSERT_TRUE(2 * period < delta + epsilon);
  ASSERT_TRUE(2 * period * overrun_ratio > delta);

  rclcpp::sleep_for(offset);
  auto two_offset = std::chrono::steady_clock::now();
  r.reset();
  ASSERT_TRUE(r.sleep());
  auto three = std::chrono::steady_clock::now();
  delta = three - two_offset;
  ASSERT_TRUE(period < delta);
  ASSERT_TRUE(period * overrun_ratio > delta);

  rclcpp::sleep_for(offset + period);
  auto four = std::chrono::steady_clock::now();
  ASSERT_FALSE(r.sleep());
  auto five = std::chrono::steady_clock::now();
  delta = five - four;
  ASSERT_TRUE(epsilon > delta);
}
