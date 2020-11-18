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

#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/domain_id.h"

#include "rclcpp/init_options.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

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
  }

  {
    auto options = rclcpp::InitOptions().auto_initialize_logging(true);
    EXPECT_TRUE(options.auto_initialize_logging());
  }

  {
    auto options = rclcpp::InitOptions().auto_initialize_logging(false);
    EXPECT_FALSE(options.auto_initialize_logging());
  }
}

TEST(TestInitOptions, test_domain_id) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::InitOptions(allocator);
  size_t domain_id = RCL_DEFAULT_DOMAIN_ID;
  EXPECT_EQ(RCL_RET_OK, rcl_get_default_domain_id(&domain_id));

  options.use_default_domain_id();
  EXPECT_EQ(domain_id, options.get_domain_id());
  options.set_domain_id(42);
  EXPECT_EQ((size_t)42, options.get_domain_id());
  options.use_default_domain_id();
  EXPECT_EQ(domain_id, options.get_domain_id());
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

TEST(TestInitOptions, constructor_rcl_init_options_init_failed) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_init_options_init, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::InitOptions(),
    std::runtime_error("failed to initialize rcl init options: error not set"));
}

TEST(TestInitOptions, constructor_rcl_init_options_copy_failed) {
  rcl_init_options_t rcl_opts;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_init_options_copy, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    new rclcpp::InitOptions(rcl_opts),
    std::runtime_error("failed to copy rcl init options: error not set"));
}

TEST(TestInitOptions, copy_constructor_rcl_init_options_copy_failed) {
  rclcpp::InitOptions options;
  rclcpp::InitOptions options2;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_init_options_copy, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    options2.operator=(options),
    std::runtime_error("failed to copy rcl init options: error not set"));
}

TEST(TestInitOptions, use_default_domain_id_rcl_get_default_domain_id_failed) {
  rclcpp::InitOptions options;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_default_domain_id, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    options.use_default_domain_id(),
    std::runtime_error("failed to get default domain id: error not set"));
}

TEST(TestInitOptions, set_domain_id_rcl_init_options_set_domain_id_failed) {
  rclcpp::InitOptions options;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_init_options_set_domain_id, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    options.set_domain_id(0),
    std::runtime_error("failed to set domain id to rcl init options: error not set"));
}

TEST(TestInitOptions, get_domain_id_rcl_init_options_get_domain_id_failed) {
  rclcpp::InitOptions options;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_init_options_get_domain_id, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    options.get_domain_id(),
    std::runtime_error("failed to get domain id from rcl init options: error not set"));
}
