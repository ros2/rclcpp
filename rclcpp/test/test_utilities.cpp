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

#include "rclcpp/utilities.hpp"

TEST(TestUtilities, remove_ros_arguments) {
  const char * const argv[] = {"process_name", "-d", "__ns:=/foo/bar",
    "__ns:=/fiz/buz", "--foo=bar", "--baz"};
  int argc = sizeof(argv) / sizeof(const char *);
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  ASSERT_EQ(4u, args.size());
  ASSERT_EQ(std::string{"process_name"}, args[0]);
  ASSERT_EQ(std::string{"-d"}, args[1]);
  ASSERT_EQ(std::string{"--foo=bar"}, args[2]);
  ASSERT_EQ(std::string{"--baz"}, args[3]);
}
