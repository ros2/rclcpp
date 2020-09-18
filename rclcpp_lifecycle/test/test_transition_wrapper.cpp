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

#include "rcutils/testing/fault_injection.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "./mocking_utils/patch.hpp"

class TestTransitionWrapper : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }
};

class TransitionDerived : public rclcpp_lifecycle::Transition
{
public:
  TransitionDerived(const uint8_t id, const std::string & label)
  : Transition(id, label) {}

  TransitionDerived(
    const uint8_t id, const std::string & label,
    rclcpp_lifecycle::State && start, rclcpp_lifecycle::State && goal)
  : Transition(id, label, std::move(start), std::move(goal)) {}
  void expose_reset()
  {
    reset();
  }
};

TEST_F(TestTransitionWrapper, empty_transition) {
  auto a = std::make_shared<rclcpp_lifecycle::Transition>(1, "my_transition");
  EXPECT_NO_THROW(a.reset());
}

TEST_F(TestTransitionWrapper, wrapper) {
  {
    rclcpp_lifecycle::Transition t(12, "no_states_set");
    EXPECT_EQ(12, t.id());
    EXPECT_STREQ("no_states_set", t.label().c_str());
  }

  {
    std::string transition_name = "no_states_set";
    rclcpp_lifecycle::Transition t(12, transition_name);
    transition_name = "not_no_states_set";
    EXPECT_EQ(12, t.id());
    EXPECT_STREQ("no_states_set", t.label().c_str());
  }

  {
    rclcpp_lifecycle::State start_state(1, "start_state");
    rclcpp_lifecycle::State goal_state(2, "goal_state");

    rclcpp_lifecycle::Transition t(
      12,
      "from_start_to_goal",
      std::move(start_state),
      std::move(goal_state));

    EXPECT_EQ(12, t.id());
    EXPECT_FALSE(t.label().empty());
    EXPECT_STREQ("from_start_to_goal", t.label().c_str());
  }
}

TEST_F(TestTransitionWrapper, copy_constructor) {
  auto a = std::make_shared<rclcpp_lifecycle::Transition>(1, "my_transition");
  rclcpp_lifecycle::Transition b(*a);

  a.reset();

  EXPECT_EQ(1, b.id());
  EXPECT_STREQ("my_transition", b.label().c_str());
}

TEST_F(TestTransitionWrapper, assignment_operator) {
  rclcpp_lifecycle::State start_state(1, "start_state");
  rclcpp_lifecycle::State goal_state(2, "goal_state");
  auto a = std::make_shared<rclcpp_lifecycle::Transition>(
    1, "one", std::move(start_state),
    std::move(goal_state));
  *a = *a;
  EXPECT_EQ(1, a->id());
  EXPECT_STREQ("one", a->label().c_str());

  auto b = std::make_shared<rclcpp_lifecycle::Transition>(2, "two");
  *b = *a;

  a.reset();

  EXPECT_EQ(1, b->id());
  EXPECT_STREQ("one", b->label().c_str());
  EXPECT_STREQ("start_state", b->start_state().label().c_str());
  EXPECT_STREQ("goal_state", b->goal_state().label().c_str());
  EXPECT_EQ(1, b->start_state().id());
  EXPECT_EQ(2, b->goal_state().id());
}

TEST_F(TestTransitionWrapper, exceptions) {
  rcl_lifecycle_transition_t * null_handle = nullptr;
  EXPECT_THROW((void)rclcpp_lifecycle::Transition(null_handle), std::runtime_error);

  rclcpp_lifecycle::State start_state(1, "start_state");
  rclcpp_lifecycle::State goal_state(2, "goal_state");
  auto a = std::make_shared<TransitionDerived>(
    1, "one", std::move(start_state),
    std::move(goal_state));

  a->expose_reset();
  EXPECT_THROW(a->start_state(), std::runtime_error);
  EXPECT_THROW(a->goal_state(), std::runtime_error);
  EXPECT_THROW(a->id(), std::runtime_error);
  EXPECT_THROW(a->label(), std::runtime_error);

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_transition_init, RCL_RET_ERROR);

    EXPECT_THROW(
      std::make_shared<TransitionDerived>(1, "one").reset(),
      std::runtime_error);

    rclcpp_lifecycle::State state1(1, "start_state");
    rclcpp_lifecycle::State state2(2, "goal_state");
    EXPECT_THROW(
      std::make_shared<TransitionDerived>(
        2, "two", std::move(start_state), std::move(goal_state)).reset(),
      std::runtime_error);
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_transition_fini, RCL_RET_ERROR);
    auto transition1 = std::make_shared<TransitionDerived>(1, "one");
    EXPECT_NO_THROW(transition1->expose_reset());

    rclcpp_lifecycle::State state1(1, "start_state");
    rclcpp_lifecycle::State state2(2, "goal_state");
    auto transition2 =
      std::make_shared<TransitionDerived>(2, "two", std::move(start_state), std::move(goal_state));
    EXPECT_NO_THROW(transition2->expose_reset());
  }

  RCUTILS_FAULT_INJECTION_TEST(
  {
    std::shared_ptr<TransitionDerived> transition = nullptr;
    try {
      transition = std::make_shared<TransitionDerived>(1, "one");
    } catch (...) {
    }
    if (nullptr != transition) {
      EXPECT_NO_THROW(transition->expose_reset());
    }
  });

  RCUTILS_FAULT_INJECTION_TEST(
  {
    std::shared_ptr<TransitionDerived> transition = nullptr;
    try {
      {
        // These will fail due to failed allocations
        rclcpp_lifecycle::State state1(1, "start_state");
        rclcpp_lifecycle::State state2(2, "goal_state");

        // Failed allocations and failed rcl init functions
        transition = std::make_shared<TransitionDerived>(
          2, "two", std::move(state1), std::move(state2));
      }
    } catch (...) {
    }

    if (nullptr != transition) {
      EXPECT_NO_THROW(transition->expose_reset());
    }
  });

  RCUTILS_FAULT_INJECTION_TEST(
  {
    try {
      // These will fail due to failed allocations
      rclcpp_lifecycle::State state1(1, "start_state");
      rclcpp_lifecycle::State state2(2, "goal_state");

      // Failed allocations and failed rcl init functions
      auto a = std::make_shared<TransitionDerived>(2, "two", std::move(state1), std::move(state2));
      auto b = std::make_shared<TransitionDerived>(3, "three");
      *b = *a;
    } catch (...) {
    }
  });
}
