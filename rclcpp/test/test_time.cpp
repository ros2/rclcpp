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

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/time.hpp"

TEST(TestRate, rate_basics) {
  using builtin_interfaces::msg::Time;
  Time ros_now = rclcpp::time::now<RCL_ROS_TIME>();
  EXPECT_NE(0, ros_now.sec);
  EXPECT_NE(0, ros_now.nanosec);

  Time system_now = rclcpp::time::now<RCL_SYSTEM_TIME>();
  EXPECT_NE(0, system_now.sec);
  EXPECT_NE(0, system_now.nanosec);

  Time steady_now = rclcpp::time::now<RCL_STEADY_TIME>();
  EXPECT_NE(0, steady_now.sec);
  EXPECT_NE(0, steady_now.nanosec);

  // default
  Time default_now = rclcpp::time::now();
  EXPECT_NE(0, default_now.sec);
  EXPECT_NE(0, default_now.nanosec);
}
