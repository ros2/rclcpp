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

#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"

class TestGuardCondition : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
 * Testing normal construction and destruction.
 */
TEST_F(TestGuardCondition, construction_and_destruction) {
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    (void)gc;
  }

  {
    // invalid context (nullptr)
    ASSERT_THROW(
    {
      auto gc = std::make_shared<rclcpp::GuardCondition>(nullptr);
      (void)gc;
    }, std::invalid_argument);
  }

  {
    // invalid context (uninitialized)
    auto context = std::make_shared<rclcpp::Context>();
    ASSERT_THROW(
    {
      auto gc = std::make_shared<rclcpp::GuardCondition>(context);
      (void)gc;
    }, rclcpp::exceptions::RCLInvalidArgument);
  }

  {
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    // This just logs an error on destruction
    EXPECT_NO_THROW(gc.reset());
  }
}

/*
 * Testing context accessor.
 */
TEST_F(TestGuardCondition, get_context) {
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    gc->get_context();
  }
}

/*
 * Testing rcl guard condition accessor.
 */
TEST_F(TestGuardCondition, get_rcl_guard_condition) {
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    gc->get_rcl_guard_condition();
  }
}

/*
 * Testing tigger method.
 */
TEST_F(TestGuardCondition, trigger) {
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    EXPECT_NO_THROW(gc->trigger());

    {
      auto mock = mocking_utils::patch_and_return(
        "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
      auto gc = std::make_shared<rclcpp::GuardCondition>();
      EXPECT_THROW(gc->trigger(), rclcpp::exceptions::RCLError);
    }
  }
}
