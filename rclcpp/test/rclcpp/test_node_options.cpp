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


TEST(TestNodeOptions, use_global_arguments) {
  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    EXPECT_FALSE(options.use_global_arguments());
    EXPECT_FALSE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions().use_global_arguments(true);
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
    options.use_global_arguments(false);
    EXPECT_FALSE(options.use_global_arguments());
    EXPECT_FALSE(options.get_rcl_node_options()->use_global_arguments);
    options.use_global_arguments(true);
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }
}
