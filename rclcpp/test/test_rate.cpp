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
   Tests that funcion_traits calculates arity of several functors.
 */
TEST(TestRate, rate_basics) {
  auto period = std::chrono::milliseconds(10);
  auto delta = std::chrono::milliseconds(1);

  auto start = std::chrono::system_clock::now();
  rclcpp::rate::Rate r(period);
  ASSERT_FALSE(r.is_steady());
  r.sleep();
  auto one = std::chrono::system_clock::now();
  ASSERT_TRUE(period - delta < one - start);
  ASSERT_TRUE(period + delta > one - start);

  rclcpp::utilities::sleep_for(delta * 4);
  r.sleep();
  auto two = std::chrono::system_clock::now();

  ASSERT_TRUE(period - delta < two - one);
  ASSERT_TRUE(period + delta > two - one);

  rclcpp::utilities::sleep_for(delta * 4);
  r.reset();
  r.sleep();
  auto three = std::chrono::system_clock::now();
  ASSERT_TRUE(period + 3 * delta < three - two);
  ASSERT_TRUE(period + 5 * delta > three - two);
}

TEST(TestRate, wallrate_basics) {
  auto period = std::chrono::milliseconds(10);
  auto delta = std::chrono::milliseconds(1);

  auto start = std::chrono::system_clock::now();
  rclcpp::rate::WallRate r(period);
  ASSERT_TRUE(r.is_steady());
  r.sleep();
  auto one = std::chrono::system_clock::now();
  ASSERT_TRUE(period - delta < one - start);
  ASSERT_TRUE(period + delta > one - start);

  rclcpp::utilities::sleep_for(delta * 4);
  r.sleep();
  auto two = std::chrono::system_clock::now();

  ASSERT_TRUE(period - delta < two - one);
  ASSERT_TRUE(period + delta > two - one);

  rclcpp::utilities::sleep_for(delta * 4);
  r.reset();
  r.sleep();
  auto three = std::chrono::system_clock::now();
  ASSERT_TRUE(period + 3 * delta < three - two);
  ASSERT_TRUE(period + 5 * delta > three - two);
}
