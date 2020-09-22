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

#include <memory>
#include <string>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

class TestNodeBase : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, >)

TEST_F(TestNodeBase, construct_from_node)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
  // the proper type is being tested and covered.
  auto * node_base =
    dynamic_cast<rclcpp::node_interfaces::NodeBase *>(node->get_node_base_interface().get());
  ASSERT_NE(nullptr, node_base);

  EXPECT_STREQ("node", node_base->get_name());
  EXPECT_STREQ("/ns", node_base->get_namespace());

  EXPECT_STREQ("/ns/node", node_base->get_fully_qualified_name());
  EXPECT_NE(nullptr, node_base->get_context());
  EXPECT_NE(nullptr, node_base->get_rcl_node_handle());
  EXPECT_NE(nullptr, node_base->get_shared_rcl_node_handle());

  const auto * const_node_base = node_base;
  EXPECT_NE(nullptr, const_node_base->get_rcl_node_handle());
  EXPECT_NE(nullptr, const_node_base->get_shared_rcl_node_handle());
}

TEST_F(TestNodeBase, construct_destruct_rcl_guard_condition_init_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_guard_condition_init, RCL_RET_ERROR);
  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_error) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_ERROR);

  // This function is called only if rcl_node_init fails, so both mocked functions are required
  // This just logs an error, so behavior shouldn't change
  auto mock_guard_condition_fini = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);

  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_node_name) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAME);

  // `rmw_validate_node_name` is only called if `rcl_node_init` returns INVALID_NAME
  auto mock_validate_node_name = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_node_name, RMW_RET_ERROR);

  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_node_name_invalid_argument) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAME);

  // `rmw_validate_node_name` is only called if `rcl_node_init` returns INVALID_NAME
  auto mock_validate_node_name = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_node_name, RMW_RET_INVALID_ARGUMENT);

  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLInvalidArgument);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_node_name_valid_rmw_node_name) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAME);

  // `rmw_validate_node_name` is only called if `rcl_node_init` returns INVALID_NAME
  auto mock = mocking_utils::patch(
    "lib:rclcpp", rmw_validate_node_name, [](const char *, int * validation_result, size_t *)
    {
      *validation_result = RMW_NODE_NAME_VALID;
      return RMW_RET_OK;
    });

  RCLCPP_EXPECT_THROW_EQ(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    std::runtime_error("valid rmw node name but invalid rcl node name"));
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_namespace) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAMESPACE);

  // `rmw_validate_namespace` is only called if `rcl_node_init` returns INVALID_NAMESPACE
  auto mock_validate_namespace = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_namespace, RMW_RET_ERROR);

  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_namespace_rmw_invalid_argument) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAMESPACE);

  // `rmw_validate_namespace` is only called if `rcl_node_init` returns INVALID_NAMESPACE
  auto mock_validate_namespace = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_validate_namespace, RMW_RET_INVALID_ARGUMENT);

  EXPECT_THROW(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    rclcpp::exceptions::RCLInvalidArgument);
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_rcl_invalid_namespace_valid_rmw_namespace) {
  auto mock_node_init = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_node_init, RCL_RET_NODE_INVALID_NAMESPACE);

  // `rmw_validate_namespace` is only called if `rcl_node_init` returns INVALID_NAMESPACE
  auto mock = mocking_utils::patch(
    "lib:rclcpp", rmw_validate_namespace, [](const char *, int * validation_result, size_t *)
    {
      *validation_result = RMW_NAMESPACE_VALID;
      return RMW_RET_OK;
    });

  RCLCPP_EXPECT_THROW_EQ(
    std::make_shared<rclcpp::Node>("node", "ns").reset(),
    std::runtime_error("valid rmw node namespace but invalid rcl node namespace"));
}

TEST_F(TestNodeBase, construct_destruct_rcl_node_init_fini_error) {
  auto mock_node_fini = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_node_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(std::make_shared<rclcpp::Node>("node", "ns").reset());
}

TEST_F(TestNodeBase, construct_destruct_rcl_guard_condition_fini_error) {
  auto mock_node_fini = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(std::make_shared<rclcpp::Node>("node", "ns").reset());
}
