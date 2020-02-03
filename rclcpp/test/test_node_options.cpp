// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rcl/allocator.h"
#include "rcl/arguments.h"
#include "rcl/remap.h"

#include "rclcpp/node_options.hpp"


TEST(TestNodeOptions, ros_args_only) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator)
    .arguments({"--ros-args", "-r", "__node:=some_node", "-r", "__ns:=/some_ns"});

  const rcl_node_options_t * rcl_options = options.get_rcl_node_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed(&rcl_options->arguments));
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed_ros(&rcl_options->arguments));

  char * node_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_name(
      &rcl_options->arguments, nullptr, "my_node", allocator, &node_name));
  ASSERT_TRUE(node_name != nullptr);
  EXPECT_STREQ("some_node", node_name);
  allocator.deallocate(node_name, allocator.state);

  char * namespace_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_namespace(
      &rcl_options->arguments, nullptr, "my_ns", allocator, &namespace_name));
  ASSERT_TRUE(namespace_name != nullptr);
  EXPECT_STREQ("/some_ns", namespace_name);
  allocator.deallocate(namespace_name, allocator.state);
}

TEST(TestNodeOptions, ros_args_and_non_ros_args) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator).arguments(
  {
    "--non-ros-flag", "--ros-args", "-r", "__node:=some_node",
    "-r", "__ns:=/some_ns", "--", "non-ros-arg"});

  const rcl_node_options_t * rcl_options = options.get_rcl_node_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed_ros(&rcl_options->arguments));
  ASSERT_EQ(2, rcl_arguments_get_count_unparsed(&rcl_options->arguments));

  char * node_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_name(
      &rcl_options->arguments, nullptr, "my_node", allocator, &node_name));
  ASSERT_TRUE(node_name != nullptr);
  EXPECT_STREQ("some_node", node_name);
  allocator.deallocate(node_name, allocator.state);

  char * namespace_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_namespace(
      &rcl_options->arguments, nullptr, "my_ns", allocator, &namespace_name));
  ASSERT_TRUE(namespace_name != nullptr);
  EXPECT_STREQ("/some_ns", namespace_name);
  allocator.deallocate(namespace_name, allocator.state);

  int * output_indices = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_arguments_get_unparsed(
      &rcl_options->arguments, allocator, &output_indices));
  ASSERT_TRUE(output_indices != nullptr);
  const std::vector<std::string> & args = options.arguments();
  EXPECT_EQ("--non-ros-flag", args[output_indices[0]]);
  EXPECT_EQ("non-ros-arg", args[output_indices[1]]);
  allocator.deallocate(output_indices, allocator.state);
}

TEST(TestNodeOptions, bad_ros_args) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator)
    .arguments({"--ros-args", "-r", "foo:="});

  EXPECT_THROW(
    options.get_rcl_node_options(),
    rclcpp::exceptions::RCLInvalidROSArgsError);

  options.arguments({"--ros-args", "-r", "foo:=bar", "not-a-ros-arg"});
  EXPECT_THROW(
    options.get_rcl_node_options(),
    rclcpp::exceptions::UnknownROSArgsError);
}

TEST(TestNodeOptions, enable_rosout) {
  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions().enable_rosout(false);
    EXPECT_FALSE(options.enable_rosout());
    EXPECT_FALSE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions().enable_rosout(true);
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
    options.enable_rosout(false);
    EXPECT_FALSE(options.enable_rosout());
    EXPECT_FALSE(options.get_rcl_node_options()->enable_rosout);
    options.enable_rosout(true);
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }
}
