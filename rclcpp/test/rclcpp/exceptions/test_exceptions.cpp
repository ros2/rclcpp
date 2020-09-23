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

#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

TEST(TestExceptions, throw_from_rcl_error) {
  EXPECT_THROW(
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_BAD_ALLOC, ""),
    rclcpp::exceptions::RCLBadAlloc);

  EXPECT_THROW(
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, ""),
    rclcpp::exceptions::RCLInvalidArgument);

  EXPECT_THROW(
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_INVALID_ROS_ARGS, ""),
    rclcpp::exceptions::RCLInvalidROSArgsError);

  EXPECT_THROW(
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, ""),
    rclcpp::exceptions::RCLError);

  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_OK, ""),
    std::invalid_argument("ret is RCL_RET_OK"));

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_get_error_state, nullptr);
    RCLCPP_EXPECT_THROW_EQ(
      rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, ""),
      std::runtime_error("rcl error state is not set"));
  }
}

TEST(TestExceptions, basic_constructors) {
  EXPECT_STREQ("node is invalid", rclcpp::exceptions::InvalidNodeError().what());
  rcl_error_state_t error_state{"error", __FILE__, __LINE__};
  EXPECT_STREQ(
    "prefix: error not set",
    rclcpp::exceptions::RCLInvalidROSArgsError(RCL_RET_ERROR, &error_state, "prefix: ").what());
}
