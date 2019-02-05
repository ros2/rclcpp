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

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNodeWithGlobalArgs : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    const char * const args[] = {"proc", "__node:=global_node_name"};
    rclcpp::init(2, args);
  }
};

TEST_F(TestNodeWithGlobalArgs, local_arguments_before_global) {
  auto options = rclcpp::NodeOptions()
    .arguments({"__node:=local_arguments_test"});

  auto node = rclcpp::Node::make_shared("orig_name", options);
  EXPECT_STREQ("local_arguments_test", node->get_name());
}

TEST_F(TestNodeWithGlobalArgs, use_or_ignore_global_arguments) {
  {  // Don't use global args
    auto options = rclcpp::NodeOptions()
      .use_global_arguments(false);

    auto node = rclcpp::Node::make_shared("orig_name", options);
    EXPECT_STREQ("orig_name", node->get_name());
  }
  {  // Do use global args
    auto options = rclcpp::NodeOptions()
      .use_global_arguments(true);

    auto node = rclcpp::Node::make_shared("orig_name", options);
    EXPECT_STREQ("global_node_name", node->get_name());
  }
}
