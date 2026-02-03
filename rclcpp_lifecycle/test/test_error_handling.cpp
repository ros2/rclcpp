// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp_lifecycle/lifecycle_exceptions.hpp"

using namespace rclcpp_lifecycle_enhanced;  // NOLINT(build/namespaces)

class ErrorHandlingTest : public ::testing::Test
{
protected:
  rclcpp_lifecycle::State make_state(uint8_t id, const std::string & label)
  {
    return rclcpp_lifecycle::State(id, label);
  }
};

TEST_F(ErrorHandlingTest, LifecycleExceptionBasics)
{
  LifecycleException ex("Test error message");

  EXPECT_STREQ(ex.what(), "Test error message");
}

TEST_F(ErrorHandlingTest, TransitionErrorCodeToString)
{
  EXPECT_EQ(transition_error_code_to_string(TransitionErrorCode::NONE), "NONE");
  EXPECT_EQ(
    transition_error_code_to_string(TransitionErrorCode::INVALID_TRANSITION),
    "INVALID_TRANSITION");
  EXPECT_EQ(
    transition_error_code_to_string(TransitionErrorCode::CALLBACK_FAILURE),
    "CALLBACK_FAILURE");
  EXPECT_EQ(transition_error_code_to_string(TransitionErrorCode::TIMEOUT), "TIMEOUT");
  EXPECT_EQ(
    transition_error_code_to_string(TransitionErrorCode::INTERNAL_ERROR),
    "INTERNAL_ERROR");
  EXPECT_EQ(transition_error_code_to_string(TransitionErrorCode::ERROR_STATE), "ERROR_STATE");
  EXPECT_EQ(
    transition_error_code_to_string(TransitionErrorCode::PRECONDITION_FAILED),
    "PRECONDITION_FAILED");
  EXPECT_EQ(transition_error_code_to_string(TransitionErrorCode::CANCELLED), "CANCELLED");
  EXPECT_EQ(transition_error_code_to_string(TransitionErrorCode::UNKNOWN), "UNKNOWN");

  // Test with an invalid code
  auto invalid_code = static_cast<TransitionErrorCode>(255);
  EXPECT_EQ(transition_error_code_to_string(invalid_code), "UNKNOWN");
}

TEST_F(ErrorHandlingTest, LifecycleTransitionExceptionWithFullDetails)
{
  auto from_state = make_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  auto to_state = make_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  LifecycleTransitionException ex(
    "Activation failed",
    TransitionErrorCode::CALLBACK_FAILURE,
    from_state,
    to_state,
    "Check hardware connection");

  EXPECT_EQ(ex.error_code(), TransitionErrorCode::CALLBACK_FAILURE);
  EXPECT_EQ(ex.from_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(ex.to_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(ex.recovery_suggestion(), "Check hardware connection");

  // Check that the message contains key information
  std::string what_str = ex.what();
  EXPECT_NE(what_str.find("CALLBACK_FAILURE"), std::string::npos);
  EXPECT_NE(what_str.find("inactive"), std::string::npos);
  EXPECT_NE(what_str.find("active"), std::string::npos);
}

TEST_F(ErrorHandlingTest, LifecycleTransitionExceptionMinimal)
{
  LifecycleTransitionException ex(
    "Simple error",
    TransitionErrorCode::INTERNAL_ERROR);

  EXPECT_EQ(ex.error_code(), TransitionErrorCode::INTERNAL_ERROR);

  std::string what_str = ex.what();
  EXPECT_NE(what_str.find("Simple error"), std::string::npos);
  EXPECT_NE(what_str.find("INTERNAL_ERROR"), std::string::npos);
}

TEST_F(ErrorHandlingTest, InvalidStateException)
{
  auto current = make_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");

  InvalidStateException ex("activate", current, "inactive or active");

  EXPECT_EQ(ex.operation(), "activate");
  EXPECT_EQ(ex.current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(ex.required_state(), "inactive or active");

  std::string what_str = ex.what();
  EXPECT_NE(what_str.find("activate"), std::string::npos);
  EXPECT_NE(what_str.find("unconfigured"), std::string::npos);
  EXPECT_NE(what_str.find("inactive or active"), std::string::npos);
}

TEST_F(ErrorHandlingTest, TransitionTimeoutException)
{
  auto from_state = make_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  auto to_state = make_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  TransitionTimeoutException ex("configure", 5000, from_state, to_state);

  EXPECT_EQ(ex.error_code(), TransitionErrorCode::TIMEOUT);
  EXPECT_EQ(ex.transition_name(), "configure");
  EXPECT_EQ(ex.timeout_ms(), 5000);
  EXPECT_EQ(ex.from_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(ex.to_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Check recovery suggestion is provided
  EXPECT_FALSE(ex.recovery_suggestion().empty());
  EXPECT_NE(ex.recovery_suggestion().find("timeout"), std::string::npos);

  std::string what_str = ex.what();
  EXPECT_NE(what_str.find("configure"), std::string::npos);
  EXPECT_NE(what_str.find("5000"), std::string::npos);
  EXPECT_NE(what_str.find("TIMEOUT"), std::string::npos);
}

TEST_F(ErrorHandlingTest, ExceptionHierarchy)
{
  // All lifecycle exceptions should be catchable as std::exception
  try {
    throw LifecycleException("base exception");
  } catch (const std::exception & e) {
    EXPECT_STREQ(e.what(), "base exception");
  }

  // LifecycleTransitionException should be catchable as LifecycleException
  try {
    throw LifecycleTransitionException("transition failed", TransitionErrorCode::CALLBACK_FAILURE);
  } catch (const LifecycleException & e) {
    EXPECT_NE(std::string(e.what()).find("transition failed"), std::string::npos);
  }

  // InvalidStateException should be catchable as LifecycleException
  try {
    auto state = make_state(1, "test");
    throw InvalidStateException("op", state, "required");
  } catch (const LifecycleException & e) {
    EXPECT_NE(std::string(e.what()).find("op"), std::string::npos);
  }

  // TransitionTimeoutException should be catchable as LifecycleTransitionException
  try {
    auto from = make_state(1, "from");
    auto to = make_state(2, "to");
    throw TransitionTimeoutException("test", 1000, from, to);
  } catch (const LifecycleTransitionException & e) {
    EXPECT_EQ(e.error_code(), TransitionErrorCode::TIMEOUT);
  }
}

TEST_F(ErrorHandlingTest, NoexceptAccessors)
{
  auto from_state = make_state(1, "from");
  auto to_state = make_state(2, "to");

  LifecycleTransitionException ex(
    "test",
    TransitionErrorCode::CALLBACK_FAILURE,
    from_state,
    to_state,
    "suggestion");

  // All accessors should be noexcept
  EXPECT_TRUE(noexcept(ex.error_code()));
  EXPECT_TRUE(noexcept(ex.from_state()));
  EXPECT_TRUE(noexcept(ex.to_state()));
  EXPECT_TRUE(noexcept(ex.recovery_suggestion()));

  InvalidStateException inv_ex("op", from_state, "req");
  EXPECT_TRUE(noexcept(inv_ex.operation()));
  EXPECT_TRUE(noexcept(inv_ex.current_state()));
  EXPECT_TRUE(noexcept(inv_ex.required_state()));

  TransitionTimeoutException timeout_ex("trans", 1000, from_state, to_state);
  EXPECT_TRUE(noexcept(timeout_ex.transition_name()));
  EXPECT_TRUE(noexcept(timeout_ex.timeout_ms()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
