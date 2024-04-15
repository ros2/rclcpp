// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <sstream>
#include <string>

#include "rclcpp/spin_until_complete_return_code.hpp"

TEST(TestSpinUntilCompleteReturnCode, to_string) {
  EXPECT_EQ(
    "Unknown enum value (-1)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode(-1)));
  EXPECT_EQ(
    "SUCCESS (0)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode::SUCCESS));
  EXPECT_EQ(
    "INTERRUPTED (1)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode::INTERRUPTED));
  EXPECT_EQ(
    "TIMEOUT (2)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode::TIMEOUT));
  EXPECT_EQ(
    "Unknown enum value (3)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode(3)));
  EXPECT_EQ(
    "Unknown enum value (100)", rclcpp::to_string(rclcpp::SpinUntilCompleteReturnCode(100)));
}

TEST(TestSpinUntilCompleteReturnCode, ostream) {
  std::ostringstream ostream;

  ostream << rclcpp::SpinUntilCompleteReturnCode::SUCCESS;
  ASSERT_EQ("SUCCESS (0)", ostream.str());
}
