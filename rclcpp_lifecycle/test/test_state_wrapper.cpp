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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

class TestStateWrapper : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }
};

TEST_F(TestStateWrapper, wrapper) {
  {
    rclcpp_lifecycle::State state(1, "my_state");
    EXPECT_EQ(1, state.id());
    EXPECT_FALSE(state.label().empty());
    EXPECT_STREQ("my_state", state.label().c_str());
  }

  {
    std::string state_name("my_state");
    rclcpp_lifecycle::State state(1, state_name);
    state_name = "not_my_state";
    EXPECT_STREQ("my_state", state.label().c_str());
  }

  {
    rcl_lifecycle_state_t lc_state = {"my_c_state", 2, NULL, 0};
    rclcpp_lifecycle::State c_state(lc_state.id, lc_state.label);
    EXPECT_EQ(2, c_state.id());
    EXPECT_FALSE(c_state.label().empty());
    EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }

  {
    rcl_lifecycle_state_t lc_state = {"my_c_state", 2, NULL, 0};
    rclcpp_lifecycle::State c_state(&lc_state);
    EXPECT_EQ(2, c_state.id());
    EXPECT_FALSE(c_state.label().empty());
    EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }

  {
    rcl_lifecycle_state_t * lc_state =
      new rcl_lifecycle_state_t {"my_c_state", 3, NULL, 0};
    rclcpp_lifecycle::State c_state(lc_state->id, lc_state->label);
    EXPECT_EQ(3, c_state.id());
    EXPECT_FALSE(c_state.label().empty());
    EXPECT_STREQ("my_c_state", c_state.label().c_str());
    delete lc_state;
  }

  // introduces flakiness
  // unsupported behavior!
  // fails when compiled with memory sanitizer
  // {
  //  rcl_lifecycle_state_t * lc_state
  //    = new rcl_lifecycle_state_t {"my_c_state", 3, NULL, NULL, 0};
  //  rclcpp_lifecycle::State c_state(lc_state);
  //  delete lc_state;
  //  lc_state = NULL;
  //  EXPECT_EQ(3, c_state.id());
  //  EXPECT_STREQ("my_c_state", c_state.label().c_str());
  // }
}

TEST_F(TestStateWrapper, copy_constructor) {
  auto a = std::make_shared<rclcpp_lifecycle::State>(1, "my_c_state");
  rclcpp_lifecycle::State b(*a);

  a.reset();

  EXPECT_EQ(1, b.id());
  EXPECT_STREQ("my_c_state", b.label().c_str());
}

TEST_F(TestStateWrapper, assignment_operator) {
  auto a = std::make_shared<rclcpp_lifecycle::State>(1, "one");
  auto b = std::make_shared<rclcpp_lifecycle::State>(2, "two");
  *b = *a;

  a.reset();

  EXPECT_EQ(1, b->id());
  EXPECT_STREQ("one", b->label().c_str());
}

TEST_F(TestStateWrapper, assignment_operator2) {
  // Non-owning State
  rcl_lifecycle_state_t * lc_state1 =
    new rcl_lifecycle_state_t{"my_c_state1", 1, NULL, 0};
  auto non_owning_state1 = std::make_shared<rclcpp_lifecycle::State>(lc_state1);

  // Non-owning State
  rcl_lifecycle_state_t * lc_state2 =
    new rcl_lifecycle_state_t{"my_c_state2", 2, NULL, 0};
  auto non_owning_state2 = std::make_shared<rclcpp_lifecycle::State>(lc_state2);

  *non_owning_state2 = *non_owning_state1;

  EXPECT_EQ(1, non_owning_state2->id());
  EXPECT_STREQ("my_c_state1", non_owning_state2->label().c_str());

  non_owning_state1.reset();
  non_owning_state2.reset();

  delete lc_state1;
  delete lc_state2;
}

TEST_F(TestStateWrapper, assignment_operator3) {
  // Non-owning State
  rcl_lifecycle_state_t * lc_state1 =
    new rcl_lifecycle_state_t{"my_c_state1", 1, NULL, 0};
  auto non_owning_state1 = std::make_shared<rclcpp_lifecycle::State>(lc_state1);

  // owning State
  auto owning_state2 = std::make_shared<rclcpp_lifecycle::State>(2, "my_c_state2");

  *owning_state2 = *non_owning_state1;

  EXPECT_EQ(1, owning_state2->id());
  EXPECT_STREQ("my_c_state1", owning_state2->label().c_str());

  non_owning_state1.reset();
  owning_state2.reset();

  delete lc_state1;
}

TEST_F(TestStateWrapper, assignment_operator4) {
  // Non-owning State
  rcl_lifecycle_state_t * lc_state1 =
    new rcl_lifecycle_state_t{"my_c_state1", 1, NULL, 0};
  auto non_owning_state1 = std::make_shared<rclcpp_lifecycle::State>(lc_state1);

  // owning State
  auto owning_state2 = std::make_shared<rclcpp_lifecycle::State>(2, "my_c_state2");

  *non_owning_state1 = *owning_state2;

  EXPECT_EQ(2, non_owning_state1->id());
  EXPECT_STREQ("my_c_state2", non_owning_state1->label().c_str());

  non_owning_state1.reset();
  owning_state2.reset();

  delete lc_state1;
}
