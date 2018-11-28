// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/utilities.hpp"

TEST(TestInit, is_initialized) {
  EXPECT_FALSE(rclcpp::is_initialized());

  rclcpp::init(0, nullptr);

  EXPECT_TRUE(rclcpp::is_initialized());

  rclcpp::shutdown();

  EXPECT_FALSE(rclcpp::is_initialized());
}
