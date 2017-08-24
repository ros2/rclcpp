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
#include <limits>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

class TestTime : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST(TestTime, time_sources) {
  using builtin_interfaces::msg::Time;
  // TODO(Karsten1987): Fix this test once ROS_TIME is implemented
  EXPECT_ANY_THROW(rclcpp::Time::now(RCL_ROS_TIME));

  Time system_now = rclcpp::Time::now(RCL_SYSTEM_TIME);
  EXPECT_NE(0, system_now.sec);
  EXPECT_NE(0u, system_now.nanosec);

  Time steady_now = rclcpp::Time::now(RCL_STEADY_TIME);
  EXPECT_NE(0, steady_now.sec);
  EXPECT_NE(0u, steady_now.nanosec);

  // default
  Time default_now = rclcpp::Time::now();
  EXPECT_NE(0, default_now.sec);
  EXPECT_NE(0u, default_now.nanosec);
}

TEST(TestTime, conversions) {
  rclcpp::Time now = rclcpp::Time::now();
  builtin_interfaces::msg::Time now_msg = now;

  rclcpp::Time now_again = now_msg;
  EXPECT_EQ(now.nanoseconds(), now_again.nanoseconds());

  builtin_interfaces::msg::Time msg;
  msg.sec = 12345;
  msg.nanosec = 67890;

  rclcpp::Time time = msg;
  EXPECT_EQ(
    RCL_S_TO_NS(static_cast<uint64_t>(msg.sec)) + static_cast<uint64_t>(msg.nanosec),
    time.nanoseconds());
  EXPECT_EQ(static_cast<uint64_t>(msg.sec), RCL_NS_TO_S(time.nanoseconds()));

  builtin_interfaces::msg::Time negative_time_msg;
  negative_time_msg.sec = -1;
  negative_time_msg.nanosec = 1;

  EXPECT_ANY_THROW({
    rclcpp::Time negative_time = negative_time_msg;
  });

  EXPECT_ANY_THROW(rclcpp::Time(-1, 1));

  EXPECT_ANY_THROW({
    rclcpp::Time assignment(1, 2);
    assignment = negative_time_msg;
  });
}

TEST(TestTime, operators) {
  rclcpp::Time old(1, 0);
  rclcpp::Time young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);

  rclcpp::Time add = old + young;
  EXPECT_EQ(add.nanoseconds(), old.nanoseconds() + young.nanoseconds());
  EXPECT_EQ(add, old + young);

  rclcpp::Time sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), young.nanoseconds() - old.nanoseconds());
  EXPECT_EQ(sub, young - old);

  rclcpp::Time system_time(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time steady_time(0, 0, RCL_STEADY_TIME);

  EXPECT_ANY_THROW((void)(system_time == steady_time));
  EXPECT_ANY_THROW((void)(system_time <= steady_time));
  EXPECT_ANY_THROW((void)(system_time >= steady_time));
  EXPECT_ANY_THROW((void)(system_time < steady_time));
  EXPECT_ANY_THROW((void)(system_time > steady_time));
  EXPECT_ANY_THROW((void)(system_time + steady_time));
  EXPECT_ANY_THROW((void)(system_time - steady_time));

  rclcpp::Time now = rclcpp::Time::now(RCL_SYSTEM_TIME);
  rclcpp::Time later = rclcpp::Time::now(RCL_STEADY_TIME);

  EXPECT_ANY_THROW((void)(now == later));
  EXPECT_ANY_THROW((void)(now <= later));
  EXPECT_ANY_THROW((void)(now >= later));
  EXPECT_ANY_THROW((void)(now < later));
  EXPECT_ANY_THROW((void)(now > later));
  EXPECT_ANY_THROW((void)(now + later));
  EXPECT_ANY_THROW((void)(now - later));

  for (auto time_source : {RCL_ROS_TIME, RCL_SYSTEM_TIME, RCL_STEADY_TIME}) {
    rclcpp::Time time = rclcpp::Time(0, 0, time_source);
    rclcpp::Time copy_constructor_time = time;
    rclcpp::Time assignment_op_time = rclcpp::Time(1, 0, time_source);
    assignment_op_time = time;

    EXPECT_TRUE(time == copy_constructor_time);
    EXPECT_TRUE(time == assignment_op_time);
  }
}

TEST(TestTime, overflows) {
  rclcpp::Time max(std::numeric_limits<uint64_t>::max());
  rclcpp::Time one(1);

  EXPECT_THROW(max + one, std::overflow_error);
  EXPECT_THROW(one - max, std::underflow_error);
}
