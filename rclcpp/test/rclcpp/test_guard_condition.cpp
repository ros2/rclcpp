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

/*
 * Testing addition to a wait set
 */
TEST_F(TestGuardCondition, add_to_wait_set) {
  {
    {
      auto gc = std::make_shared<rclcpp::GuardCondition>();

      auto mock = mocking_utils::patch_and_return(
        "lib:rclcpp", rcl_wait_set_add_guard_condition, RCL_RET_OK);

      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_NO_THROW(gc->add_to_wait_set(&wait_set));
      EXPECT_NO_THROW(gc->add_to_wait_set(&wait_set));

      rcl_wait_set_t wait_set_2 = rcl_get_zero_initialized_wait_set();
      EXPECT_THROW(gc->add_to_wait_set(&wait_set_2), std::runtime_error);
    }

    {
      auto mock = mocking_utils::patch_and_return(
        "lib:rclcpp", rcl_wait_set_add_guard_condition, RCL_RET_ERROR);

      auto gd = std::make_shared<rclcpp::GuardCondition>();
      EXPECT_THROW(gd->add_to_wait_set(nullptr), rclcpp::exceptions::RCLError);
    }
  }
}

/*
 * Testing set on trigger callback
 */
TEST_F(TestGuardCondition, set_on_trigger_callback) {
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();

    std::atomic<size_t> c1 {0};
    auto increase_c1_cb = [&c1](size_t count_msgs) {c1 += count_msgs;};
    gc->set_on_trigger_callback(increase_c1_cb);

    EXPECT_EQ(c1.load(), 0u);
    EXPECT_NO_THROW(gc->trigger());
    EXPECT_EQ(c1.load(), 1u);

    std::atomic<size_t> c2 {0};
    auto increase_c2_cb = [&c2](size_t count_msgs) {c2 += count_msgs;};
    gc->set_on_trigger_callback(increase_c2_cb);

    EXPECT_NO_THROW(gc->trigger());
    EXPECT_EQ(c1.load(), 1u);
    EXPECT_EQ(c2.load(), 1u);

    gc->set_on_trigger_callback(nullptr);
    EXPECT_NO_THROW(gc->trigger());
    EXPECT_EQ(c1.load(), 1u);
    EXPECT_EQ(c2.load(), 1u);

    gc->set_on_trigger_callback(increase_c1_cb);
    EXPECT_EQ(c1.load(), 2u);
  }
}
