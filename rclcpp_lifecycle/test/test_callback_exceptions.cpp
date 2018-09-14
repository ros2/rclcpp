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

class TestCallbackExceptions : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

class PositiveCallbackExceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PositiveCallbackExceptionNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}

  size_t number_of_callbacks = 0;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    throw std::runtime_error("custom exception raised in configure callback");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

TEST_F(TestCallbackExceptions, positive_on_error) {
  auto test_node = std::make_shared<PositiveCallbackExceptionNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  // check if all callbacks were successfully overwritten
  EXPECT_EQ(2u, test_node->number_of_callbacks);
}

TEST_F(TestCallbackExceptions, positive_on_error_with_code) {
  auto test_node = std::make_shared<PositiveCallbackExceptionNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  test_node->configure(ret);
  EXPECT_EQ(rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR, ret);
}

class NegativeCallbackExceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NegativeCallbackExceptionNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}

  size_t number_of_callbacks = 0;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    throw std::runtime_error("custom exception raised in configure callback");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &)
  {
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
};

TEST_F(TestCallbackExceptions, negative_on_error) {
  auto test_node = std::make_shared<NegativeCallbackExceptionNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  // check if all callbacks were successfully overwritten
  EXPECT_EQ(2u, test_node->number_of_callbacks);
}

TEST_F(TestCallbackExceptions, negative_on_error_with_code) {
  auto test_node = std::make_shared<NegativeCallbackExceptionNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret;
  test_node->configure(ret);
  EXPECT_EQ(rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR, ret);
}
