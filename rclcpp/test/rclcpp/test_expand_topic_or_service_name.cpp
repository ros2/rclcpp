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

#include <stdexcept>

#include "rcl/expand_topic_name.h"
#include "rcl/validate_topic_name.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

/*
   Testing expand_topic_or_service_name.
 */
TEST(TestExpandTopicOrServiceName, normal) {
  using rclcpp::expand_topic_or_service_name;
  {
    ASSERT_EQ("/ns/chatter", expand_topic_or_service_name("chatter", "node", "/ns"));
  }
}

/*
   Testing exceptions of expand_topic_or_service_name.
 */
TEST(TestExpandTopicOrServiceName, exceptions) {
  using rclcpp::expand_topic_or_service_name;
  {
    ASSERT_THROW(
    {
      expand_topic_or_service_name("chatter", "invalid_node?", "/ns");
    }, rclcpp::exceptions::InvalidNodeNameError);
  }

  {
    ASSERT_THROW(
    {
      expand_topic_or_service_name("chatter", "node", "/invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }

  {
    ASSERT_THROW(
    {
      expand_topic_or_service_name("chatter/42invalid", "node", "/ns");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }

  {
    ASSERT_THROW(
    {
      // this one will only fail on the "full" topic name validation check
      expand_topic_or_service_name("chatter/{ns}/invalid", "node", "/ns");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }

  {
    ASSERT_THROW(
    {
      // is_service = true
      expand_topic_or_service_name("chatter/42invalid", "node", "/ns", true);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW(
    {
      // is_service = true
      // this one will only fail on the "full" topic name validation check
      expand_topic_or_service_name("chatter/{ns}/invalid", "node", "/ns", true);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

TEST(TestExpandTopicOrServiceName, rcutils_string_map_init_fail_bad_alloc) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_map_init, RCUTILS_RET_BAD_ALLOC);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::bad_alloc());
}

TEST(TestExpandTopicOrServiceName, rcutils_string_map_init_fail_other) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_map_init, RCUTILS_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("error not set"));
}

TEST(TestExpandTopicOrServiceName, rcl_get_default_topic_name_substitution_fail) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_default_topic_name_substitutions, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("error not set"));
}

TEST(TestExpandTopicOrServiceName, rcl_get_default_topic_name_substitution_and_map_fini_fail) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_default_topic_name_substitutions, RCL_RET_ERROR);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_map_fini, RCUTILS_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("error not set"));
}

TEST(TestExpandTopicOrServiceName, rcutils_string_map_fini_fail_bad_alloc) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_map_fini, RCUTILS_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("error not set"));
}

TEST(TestExpandTopicOrServiceName, rmw_valid_full_topic_name_fail_invalid_argument) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_full_topic_name, RMW_RET_INVALID_ARGUMENT);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLInvalidArgument(
      RCL_RET_INVALID_ARGUMENT, rcl_get_error_state(), "failed to validate full topic name"));
}

TEST(TestExpandTopicOrServiceName, rcl_expand_topic_name_fail) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_TOPIC_NAME_INVALID);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("topic name unexpectedly valid"));
}

TEST(TestExpandTopicOrServiceName, rcl_validate_topic_name_fail) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_TOPIC_NAME_INVALID);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_validate_topic_name, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to validate full topic name"));
}

TEST(TestExpandTopicOrServiceName, rmw_validate_node_name_fail_invalid_argument) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAME);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_node_name, RMW_RET_INVALID_ARGUMENT);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLInvalidArgument(
      RCL_RET_INVALID_ARGUMENT, rcl_get_error_state(), "failed to validate node name"));
}

TEST(TestExpandTopicOrServiceName, rmw_validate_node_name_fail_other) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAME);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_node_name, RMW_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to validate node name"));
}

TEST(TestExpandTopicOrServiceName, rmw_validate_namespace_fail_invalid_argument) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAMESPACE);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_namespace, RMW_RET_INVALID_ARGUMENT);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLInvalidArgument(
      RCL_RET_INVALID_ARGUMENT, rcl_get_error_state(), "failed to validate namespace"));
}

TEST(TestExpandTopicOrServiceName, rmw_validate_namespace_fail_other) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAMESPACE);
  auto mock2 = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_namespace, RMW_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to validate namespace"));
}

TEST(TestExpandTopicOrServiceName, rcl_expand_topic_name_fail_other) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLError(RCL_RET_ERROR, rcl_get_error_state(), "error not set"));
}

TEST(TestExpandTopicOrServiceName, rcl_expand_topic_name_fail_invalid_node_name) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAME);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("invalid rcl node name but valid rmw node name"));
}

TEST(TestExpandTopicOrServiceName, rcl_expand_topic_name_fail_invalid_node_namespace) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_expand_topic_name, RCL_RET_NODE_INVALID_NAMESPACE);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    std::runtime_error("invalid rcl namespace but valid rmw namespace"));
}

TEST(TestExpandTopicOrServiceName, rmw_validate_full_topic_name_fail_other) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_full_topic_name, RMW_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::expand_topic_or_service_name("chatter", "node", "/ns"),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to validate full topic name"));
}
