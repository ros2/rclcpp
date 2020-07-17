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

#include <cstring>
#include <exception>

#include <string>

#include "./rclcpp_gtest_macros.hpp"

#include "rcl/rcl.h"
#include "rclcpp/rclcpp.hpp"

struct NonStandardThrowable
{
  bool operator==(const NonStandardThrowable &) const
  {
    return true;
  }
};

std::ostream & operator<<(std::ostream & os, const NonStandardThrowable &)
{
  os << "NonStandardThrowable";
  return os;
}

TEST(TestGtestMacros, standard_exceptions) {
  RCLCPP_EXPECT_THROW_EQ(
    throw std::runtime_error("some runtime error"),
    std::runtime_error("some runtime error"));

  RCLCPP_EXPECT_THROW_EQ(
    throw std::invalid_argument("some invalid argument error"),
    std::invalid_argument("some invalid argument error"));

  RCLCPP_ASSERT_THROW_EQ(
    throw std::runtime_error("some runtime error"),
    std::runtime_error("some runtime error"));

  RCLCPP_ASSERT_THROW_EQ(
    throw std::invalid_argument("some invalid argument error"),
    std::invalid_argument("some invalid argument error"));
}

TEST(TestGtestMacros, standard_exceptions_not_equals) {
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  CHECK_THROW_EQ_IMPL(
    throw std::runtime_error("some runtime error"),
    std::range_error("some runtime error"),
    result);
  EXPECT_FALSE(result);

  CHECK_THROW_EQ_IMPL(
    throw std::invalid_argument("some invalid argument error"),
    std::invalid_argument("some different invalid argument error"),
    result);
  EXPECT_FALSE(result);
}

TEST(TestGTestMacros, non_standard_types) {
  RCLCPP_EXPECT_THROW_EQ(throw 0, 0);

  RCLCPP_EXPECT_THROW_EQ(throw 42, 42);

  RCLCPP_EXPECT_THROW_EQ(throw std::string("some string"), std::string("some string"));

  RCLCPP_EXPECT_THROW_EQ(throw NonStandardThrowable(), NonStandardThrowable());

  RCLCPP_ASSERT_THROW_EQ(throw 0, 0);

  RCLCPP_ASSERT_THROW_EQ(throw 42, 42);

  RCLCPP_ASSERT_THROW_EQ(throw std::string("some string"), std::string("some string"));

  RCLCPP_ASSERT_THROW_EQ(throw NonStandardThrowable(), NonStandardThrowable());
}

TEST(TestGTestMacros, non_standard_types_not_equals) {
  ::testing::AssertionResult result = ::testing::AssertionSuccess();

  CHECK_THROW_EQ_IMPL(throw 0, 1, result);
  EXPECT_FALSE(result);
  result = ::testing::AssertionSuccess();

  CHECK_THROW_EQ_IMPL(throw -42, 42, result);
  EXPECT_FALSE(result);
  result = ::testing::AssertionSuccess();

  CHECK_THROW_EQ_IMPL(throw std::string("some string"), std::string("some other string"), result);
  EXPECT_FALSE(result);
}

TEST(TestGTestMacros, rclcpp_exceptions) {
  rcutils_error_state_t rcl_error_state = {"this is some error message", __FILE__, __LINE__};
  {
    auto expected =
      rclcpp::exceptions::RCLError(RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLError(RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    RCLCPP_EXPECT_THROW_EQ(throw actual, expected);
    RCLCPP_ASSERT_THROW_EQ(throw actual, expected);
  }
  {
    auto expected =
      rclcpp::exceptions::RCLBadAlloc(RCL_RET_BAD_ALLOC, &rcl_error_state);
    auto actual =
      rclcpp::exceptions::RCLBadAlloc(RCL_RET_BAD_ALLOC, &rcl_error_state);
    RCLCPP_EXPECT_THROW_EQ(throw actual, expected);
  }
  {
    // Prefixes are not checked
    auto expected =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &rcl_error_state, "different_prefix");
    RCLCPP_EXPECT_THROW_EQ(throw actual, expected);
    RCLCPP_ASSERT_THROW_EQ(throw actual, expected);
  }
  {
    // File names are not checked
    rcutils_error_state_t different_error_state = rcl_error_state;
    std::snprintf(
      different_error_state.file, RCUTILS_ERROR_STATE_FILE_MAX_LENGTH, "different_file.cpp");
    auto expected =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &different_error_state, "exception_prefix");
    RCLCPP_EXPECT_THROW_EQ(throw actual, expected);
    RCLCPP_ASSERT_THROW_EQ(throw actual, expected);
  }
  {
    // Line numbers are not checked
    rcutils_error_state_t different_error_state = rcl_error_state;
    different_error_state.line_number += 42;
    auto expected =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &different_error_state, "exception_prefix");
    RCLCPP_EXPECT_THROW_EQ(throw actual, expected);
    RCLCPP_ASSERT_THROW_EQ(throw actual, expected);
  }
}

TEST(TestGTestMacros, rclcpp_exceptions_not_equal) {
  rcutils_error_state_t rcl_error_state = {"this is some error message", __FILE__, __LINE__};
  {
    // Check different return errors
    ::testing::AssertionResult result = ::testing::AssertionSuccess();
    auto expected =
      rclcpp::exceptions::RCLError(RCL_RET_ERROR, &rcl_error_state, "exception_prefix");

    auto actual =
      rclcpp::exceptions::RCLError(RCL_RET_BAD_ALLOC, &rcl_error_state, "exception_prefix");
    CHECK_THROW_EQ_IMPL(throw actual, expected, result);
    EXPECT_FALSE(result);
  }
  {
    // Check different error messages
    rcutils_error_state_t different_error_state = rcl_error_state;
    std::snprintf(
      different_error_state.message,
      RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH,
      "this is a different error message");
    ::testing::AssertionResult result = ::testing::AssertionSuccess();
    auto expected =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, &different_error_state, "exception_prefix");
    CHECK_THROW_EQ_IMPL(throw actual, expected, result);
    EXPECT_FALSE(result);
  }
  {
    // Check different exception types
    ::testing::AssertionResult result = ::testing::AssertionSuccess();
    auto expected =
      rclcpp::exceptions::RCLError(RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    auto actual =
      rclcpp::exceptions::RCLInvalidArgument(RCL_RET_ERROR, &rcl_error_state, "exception_prefix");
    CHECK_THROW_EQ_IMPL(throw actual, expected, result);
    EXPECT_FALSE(result);
  }
}
