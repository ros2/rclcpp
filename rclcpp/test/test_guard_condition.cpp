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
    gc->trigger();
  }
}
