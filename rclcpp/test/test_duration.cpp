// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"


using namespace std::chrono_literals;

class TestDuration : public ::testing::Test
{
};

// TEST(TestDuration, conversions) {
// TODO(tfoote) Implement conversion methods
// }

TEST(TestDuration, operators) {
  rclcpp::Duration old(1, 0);
  rclcpp::Duration young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);

  rclcpp::Duration add = old + young;
  EXPECT_EQ(add.nanoseconds(), (rcl_duration_value_t)(old.nanoseconds() + young.nanoseconds()));
  EXPECT_EQ(add, old + young);

  rclcpp::Duration sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), (rcl_duration_value_t)(young.nanoseconds() - old.nanoseconds()));
  EXPECT_EQ(sub, young - old);

  rclcpp::Duration scale = old * 3;
  EXPECT_EQ(scale.nanoseconds(), (rcl_duration_value_t)(old.nanoseconds() * 3));

  rclcpp::Duration time = rclcpp::Duration(0, 0);
  rclcpp::Duration copy_constructor_duration(time);
  rclcpp::Duration assignment_op_duration = rclcpp::Duration(1, 0);
  assignment_op_duration = time;

  EXPECT_TRUE(time == copy_constructor_duration);
  EXPECT_TRUE(time == assignment_op_duration);
}

TEST(TestDuration, chrono_overloads) {
  int64_t ns = 123456789l;
  auto chrono_ns = std::chrono::nanoseconds(ns);
  auto d1 = rclcpp::Duration(ns);
  auto d2 = rclcpp::Duration(chrono_ns);
  auto d3 = rclcpp::Duration(123456789ns);
  EXPECT_EQ(d1, d2);
  EXPECT_EQ(d1, d3);
  EXPECT_EQ(d2, d3);
}

TEST(TestDuration, overflows) {
  rclcpp::Duration max(std::numeric_limits<rcl_duration_value_t>::max());
  rclcpp::Duration min(std::numeric_limits<rcl_duration_value_t>::min());

  rclcpp::Duration one(1);
  rclcpp::Duration negative_one(-1);

  EXPECT_THROW(max + one, std::overflow_error);
  EXPECT_THROW(min - one, std::underflow_error);
  EXPECT_THROW(negative_one + min, std::underflow_error);
  EXPECT_THROW(negative_one - max, std::underflow_error);

  rclcpp::Duration base_d = max * 0.3;
  EXPECT_THROW(base_d * 4, std::overflow_error);
  EXPECT_THROW(base_d * (-4), std::underflow_error);

  rclcpp::Duration base_d_neg = max * (-0.3);
  EXPECT_THROW(base_d_neg * (-4), std::overflow_error);
  EXPECT_THROW(base_d_neg * 4, std::underflow_error);
}

TEST(TestDuration, negative_duration) {
  rclcpp::Duration assignable_duration = rclcpp::Duration(0) - rclcpp::Duration(5, 0);

  // Since number is smaller than -INT_MAX, must explicitly set the type for windows.
  const int64_t expected_value_1 = -5000000000;
  EXPECT_EQ(expected_value_1, assignable_duration.nanoseconds());

  builtin_interfaces::msg::Duration duration_msg;
  duration_msg.sec = -4;
  duration_msg.nanosec = 250000000;

  assignable_duration = duration_msg;
  const int64_t expected_value_2 = -3750000000;
  EXPECT_EQ(expected_value_2, assignable_duration.nanoseconds());
}
