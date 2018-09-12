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

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

struct GoodMood
{
  static constexpr uint8_t cb_ret =
    lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
};
struct BadMood
{
  static constexpr uint8_t cb_ret =
    lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
};

class TestDefaultStateMachine : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmptyLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}
};

template<class Mood = GoodMood>
class MoodyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MoodyLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}

  size_t number_of_callbacks = 0;

protected:
  rcl_lifecycle_transition_key_t
  on_configure(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CONFIGURING, get_current_state().id());
    ++number_of_callbacks;
    return {Mood::cb_ret, ""};
  }

  rcl_lifecycle_transition_key_t
  on_activate(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return {Mood::cb_ret, ""};
  }

  rcl_lifecycle_transition_key_t
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_DEACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return {Mood::cb_ret, ""};
  }

  rcl_lifecycle_transition_key_t
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CLEANINGUP, get_current_state().id());
    ++number_of_callbacks;
    return {Mood::cb_ret, ""};
  }

  rcl_lifecycle_transition_key_t
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_SHUTTINGDOWN, get_current_state().id());
    ++number_of_callbacks;
    return {Mood::cb_ret, ""};
  }

  rcl_lifecycle_transition_key_t
  on_error(const rclcpp_lifecycle::State &);
};

template<>
rcl_lifecycle_transition_key_t
MoodyLifecycleNode<GoodMood>::on_error(const rclcpp_lifecycle::State &)
{
  EXPECT_EQ(State::TRANSITION_STATE_ERRORPROCESSING, get_current_state().id());
  ADD_FAILURE();
  return {lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR, ""};
}
template<>
rcl_lifecycle_transition_key_t
MoodyLifecycleNode<BadMood>::on_error(const rclcpp_lifecycle::State &)
{
  EXPECT_EQ(State::TRANSITION_STATE_ERRORPROCESSING, get_current_state().id());
  ++number_of_callbacks;
  return {lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ""};
}

TEST_F(TestDefaultStateMachine, empty_initializer) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  EXPECT_STREQ("testnode", test_node->get_name());
  EXPECT_STREQ("/", test_node->get_namespace());
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
}

TEST_F(TestDefaultStateMachine, trigger_transition) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  ASSERT_EQ(State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_SHUTDOWN)).id());
}

TEST_F(TestDefaultStateMachine, trigger_transition_with_error_code) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  rcl_lifecycle_transition_key_t reset_key = {
    lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR, ""
  };
  rcl_lifecycle_transition_key_t ret = reset_key;

  test_node->configure(ret);
  EXPECT_EQ(lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ret.id);
  ret = reset_key;

  test_node->activate(ret);
  EXPECT_EQ(lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ret.id);
  ret = reset_key;

  test_node->deactivate(ret);
  EXPECT_EQ(lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ret.id);
  ret = reset_key;

  test_node->cleanup(ret);
  EXPECT_EQ(lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ret.id);
  ret = reset_key;

  test_node->shutdown(ret);
  EXPECT_EQ(lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS, ret.id);
}

TEST_F(TestDefaultStateMachine, good_mood) {
  auto test_node = std::make_shared<MoodyLifecycleNode<GoodMood>>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_SHUTDOWN)).id());

  // check if all callbacks were successfully overwritten
  EXPECT_EQ(static_cast<size_t>(5), test_node->number_of_callbacks);
}

TEST_F(TestDefaultStateMachine, bad_mood) {
  auto test_node = std::make_shared<MoodyLifecycleNode<BadMood>>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());

  // check if all callbacks were successfully overwritten
  EXPECT_EQ(static_cast<size_t>(1), test_node->number_of_callbacks);
}

TEST_F(TestDefaultStateMachine, lifecycle_subscriber) {
  auto test_node = std::make_shared<MoodyLifecycleNode<GoodMood>>("testnode");

  auto cb = [](const std::shared_ptr<lifecycle_msgs::msg::State> msg) {(void) msg;};
  auto lifecycle_sub = test_node->create_subscription<lifecycle_msgs::msg::State>("~/empty", cb);

  SUCCEED();
}
