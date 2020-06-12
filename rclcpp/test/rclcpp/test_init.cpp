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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"

#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

TEST(TestInit, is_initialized) {
  EXPECT_FALSE(rclcpp::is_initialized());

  rclcpp::init(0, nullptr);

  EXPECT_TRUE(rclcpp::is_initialized());

  rclcpp::shutdown();

  EXPECT_FALSE(rclcpp::is_initialized());
}

TEST(TestInit, initialize_with_ros_args) {
  EXPECT_FALSE(rclcpp::is_initialized());

  rclcpp::init(0, nullptr);

  EXPECT_TRUE(rclcpp::is_initialized());

  rclcpp::shutdown();

  EXPECT_FALSE(rclcpp::is_initialized());
}

TEST(TestInit, initialize_with_unknown_ros_args) {
  EXPECT_FALSE(rclcpp::is_initialized());

  const char * const argv[] = {"--ros-args", "unknown"};
  const int argc = static_cast<int>(sizeof(argv) / sizeof(const char *));
  EXPECT_THROW({rclcpp::init(argc, argv);}, rclcpp::exceptions::UnknownROSArgsError);

  EXPECT_FALSE(rclcpp::is_initialized());
}

#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
