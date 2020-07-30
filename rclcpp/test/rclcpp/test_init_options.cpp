// Copyright 2020 Open Source Robotics Foundation, Inc.
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
#include "rcl/domain_id.h"

#include "rclcpp/init_options.hpp"


TEST(TestInitOptions, test_construction) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::InitOptions(allocator);
  const rcl_init_options_t * rcl_options = options.get_rcl_init_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_TRUE(rcl_options->impl != nullptr);

  {
    auto options_copy = rclcpp::InitOptions(options);
    const rcl_init_options_t * rcl_options_copy = options_copy.get_rcl_init_options();
    ASSERT_TRUE(rcl_options_copy != nullptr);
    ASSERT_TRUE(rcl_options_copy->impl != nullptr);
  }

  {
    auto options_copy = options;
    const rcl_init_options_t * rcl_options_copy = options_copy.get_rcl_init_options();
    ASSERT_TRUE(rcl_options_copy != nullptr);
    ASSERT_TRUE(rcl_options_copy->impl != nullptr);
  }
}

TEST(TestInitOptions, test_initialize_logging) {
  {
    auto options = rclcpp::InitOptions();
    EXPECT_TRUE(options.auto_initialize_logging());
    const rcl_init_options_t * rcl_options = options.get_rcl_init_options();
    ASSERT_TRUE(rcl_options != nullptr);
    ASSERT_TRUE(rcl_options->impl != nullptr);
  }

  {
    auto options = rclcpp::InitOptions().auto_initialize_logging(true);
    EXPECT_TRUE(options.auto_initialize_logging());
    const rcl_init_options_t * rcl_options = options.get_rcl_init_options();
    ASSERT_TRUE(rcl_options != nullptr);
    ASSERT_TRUE(rcl_options->impl != nullptr);
  }

  {
    auto options = rclcpp::InitOptions().auto_initialize_logging(false);
    EXPECT_FALSE(options.auto_initialize_logging());
    const rcl_init_options_t * rcl_options = options.get_rcl_init_options();
    ASSERT_TRUE(rcl_options != nullptr);
    ASSERT_TRUE(rcl_options->impl != nullptr);
  }
}

TEST(TestInitOptions, test_domain_id) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::InitOptions(allocator);
  const rcl_init_options_t * rcl_options = options.get_rcl_init_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_TRUE(rcl_options->impl != nullptr);

  options.use_default_domain_id();
  EXPECT_EQ(RCL_DEFAULT_DOMAIN_ID, options.get_domain_id());
  options.set_domain_id(42);
  EXPECT_EQ((size_t)42, options.get_domain_id());
  options.use_default_domain_id();
  EXPECT_EQ(RCL_DEFAULT_DOMAIN_ID, options.get_domain_id());
}
