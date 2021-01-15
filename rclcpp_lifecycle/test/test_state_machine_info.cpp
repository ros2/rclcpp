// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class TestStateMachineInfo : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestStateMachineInfo, available_states) {
  auto test_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testnode");
  std::vector<rclcpp_lifecycle::State> available_states =
    test_node->get_available_states();
  EXPECT_EQ(11u, available_states.size());

  // Primary States
  EXPECT_EQ(0, available_states[0].id());  // unknown
  EXPECT_EQ(1, available_states[1].id());  // unconfigured
  EXPECT_EQ(2, available_states[2].id());  // inactive
  EXPECT_EQ(3, available_states[3].id());  // active
  EXPECT_EQ(4, available_states[4].id());  // finalized

  // Transition States
  EXPECT_EQ(10, available_states[5].id());  // configuring
  EXPECT_EQ(11, available_states[6].id());  // cleaningup
  EXPECT_EQ(12, available_states[7].id());  // shuttingdown
  EXPECT_EQ(13, available_states[8].id());  // activating
  EXPECT_EQ(14, available_states[9].id());  // deactivating
  EXPECT_EQ(15, available_states[10].id());  // errorprocessing
}

TEST_F(TestStateMachineInfo, available_transitions) {
  auto test_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testnode");
  std::vector<rclcpp_lifecycle::Transition> current_available_transitions;

  // PRIMARY_STATE_UNCONFIGURED
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(2u, current_available_transitions.size());
  EXPECT_EQ(
    current_available_transitions[0].id(),
    Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    current_available_transitions[1].id(),
    Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  current_available_transitions.clear();

  // PRIMARY_STATE_INACTIVE
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(3u, current_available_transitions.size());
  EXPECT_EQ(
    current_available_transitions[0].id(),
    Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(
    current_available_transitions[1].id(),
    Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(
    current_available_transitions[2].id(),
    Transition::TRANSITION_INACTIVE_SHUTDOWN);
  current_available_transitions.clear();

  // PRIMARY_STATE_ACTIVE
  ASSERT_EQ(
    State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(2u, current_available_transitions.size());
  EXPECT_EQ(
    current_available_transitions[0].id(),
    Transition::TRANSITION_DEACTIVATE);
  EXPECT_EQ(
    current_available_transitions[1].id(),
    Transition::TRANSITION_ACTIVE_SHUTDOWN);
  current_available_transitions.clear();

  // PRIMARY_STATE_INACTIVE
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(3u, current_available_transitions.size());
  EXPECT_EQ(
    current_available_transitions[0].id(),
    Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(
    current_available_transitions[1].id(),
    Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(
    current_available_transitions[2].id(),
    Transition::TRANSITION_INACTIVE_SHUTDOWN);
  current_available_transitions.clear();

  // PRIMARY_STATE_UNCONFIGURED
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(2u, current_available_transitions.size());
  EXPECT_EQ(
    current_available_transitions[0].id(),
    Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    current_available_transitions[1].id(),
    Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  current_available_transitions.clear();

  // PRIMARY_STATE_FINALIZED
  ASSERT_EQ(
    State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
  current_available_transitions = test_node->get_available_transitions();
  EXPECT_EQ(0u, current_available_transitions.size());
  current_available_transitions.clear();
}

TEST_F(TestStateMachineInfo, transition_graph) {
  auto test_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testnode");
  std::vector<rclcpp_lifecycle::Transition> available_transitions =
    test_node->get_transition_graph();
  EXPECT_EQ(25u, available_transitions.size());
  for (rclcpp_lifecycle::Transition & transition : available_transitions) {
    EXPECT_FALSE(transition.label().empty());

    EXPECT_TRUE(
      transition.start_state().id() <= 4 ||
      (transition.start_state().id() >= 10 &&
      (transition.start_state().id() <= 15)));
    EXPECT_FALSE(transition.start_state().label().empty());

    EXPECT_TRUE(
      transition.goal_state().id() <= 4 ||
      (transition.goal_state().id() >= 10 &&
      (transition.goal_state().id() <= 15)));
    EXPECT_FALSE(transition.goal_state().label().empty());
  }
}
