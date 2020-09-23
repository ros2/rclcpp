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

#include "rclcpp/future_return_code.hpp"

TEST(TestFutureReturnCode, to_string) {
  EXPECT_EQ(
    std::string("Unknown enum value (-1)"), rclcpp::to_string(rclcpp::FutureReturnCode(-1)));
  EXPECT_EQ(
    std::string("SUCCESS (0)"), rclcpp::to_string(rclcpp::FutureReturnCode::SUCCESS));
  EXPECT_EQ(
    std::string("INTERRUPTED (1)"), rclcpp::to_string(rclcpp::FutureReturnCode::INTERRUPTED));
  EXPECT_EQ(
    std::string("TIMEOUT (2)"), rclcpp::to_string(rclcpp::FutureReturnCode::TIMEOUT));
  EXPECT_EQ(
    std::string("Unknown enum value (3)"), rclcpp::to_string(rclcpp::FutureReturnCode(3)));
  EXPECT_EQ(
    std::string("Unknown enum value (100)"), rclcpp::to_string(rclcpp::FutureReturnCode(100)));
}
