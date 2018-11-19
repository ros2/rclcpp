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

#include <string>
#include <memory>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"

TEST(TestUtilities, remove_ros_arguments) {
  const char * const argv[] = {"process_name", "-d", "__ns:=/foo/bar",
    "__ns:=/fiz/buz", "--foo=bar", "--baz"};
  int argc = sizeof(argv) / sizeof(const char *);
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  ASSERT_EQ(4u, args.size());
  ASSERT_EQ(std::string{"process_name"}, args[0]);
  ASSERT_EQ(std::string{"-d"}, args[1]);
  ASSERT_EQ(std::string{"--foo=bar"}, args[2]);
  ASSERT_EQ(std::string{"--baz"}, args[3]);
}

TEST(TestUtilities, remove_ros_arguments_null) {
  // In the case of a C executable, we would expect to get
  // argc=1 and argv = ["process_name"], so this is an invalid input.
  ASSERT_THROW({
    rclcpp::remove_ros_arguments(0, nullptr);
  }, rclcpp::exceptions::RCLErrorBase);
}

TEST(TestUtilities, init_with_args) {
  const char * const argv[] = {"process_name"};
  int argc = sizeof(argv) / sizeof(const char *);
  auto other_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  ASSERT_EQ(1u, other_args.size());
  ASSERT_EQ(std::string{"process_name"}, other_args[0]);

  EXPECT_TRUE(rclcpp::ok());
  rclcpp::shutdown();
}

TEST(TestUtilities, multi_init) {
  auto context1 = std::make_shared<rclcpp::contexts::default_context::DefaultContext>();
  auto context2 = std::make_shared<rclcpp::contexts::default_context::DefaultContext>();
  EXPECT_TRUE(rclcpp::ok(context1));
  EXPECT_TRUE(rclcpp::ok(context2));
  rclcpp::shutdown(context1);
  rclcpp::shutdown(context2);
}
