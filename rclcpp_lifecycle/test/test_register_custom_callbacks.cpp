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

class TestRegisterCustomCallbacks : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

class CustomLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CustomLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}

  size_t number_of_callbacks = 0;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    ADD_FAILURE();
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    ADD_FAILURE();
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    ADD_FAILURE();
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    ADD_FAILURE();
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    ADD_FAILURE();
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Custom callbacks

public:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_custom_configure(const rclcpp_lifecycle::State & previous_state)
  {
    EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, previous_state.id());
    EXPECT_EQ(State::TRANSITION_STATE_CONFIGURING, get_current_state().id());
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_custom_activate(const rclcpp_lifecycle::State & previous_state)
  {
    EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, previous_state.id());
    EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_custom_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, previous_state.id());
    EXPECT_EQ(State::TRANSITION_STATE_DEACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_custom_cleanup(const rclcpp_lifecycle::State & previous_state)
  {
    EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, previous_state.id());
    EXPECT_EQ(State::TRANSITION_STATE_CLEANINGUP, get_current_state().id());
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_custom_shutdown(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_SHUTTINGDOWN, get_current_state().id());
    ++number_of_callbacks;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

TEST_F(TestRegisterCustomCallbacks, custom_callbacks) {
  auto test_node = std::make_shared<CustomLifecycleNode>("testnode");

  test_node->register_on_configure(std::bind(&CustomLifecycleNode::on_custom_configure,
    test_node.get(), std::placeholders::_1));
  test_node->register_on_cleanup(std::bind(&CustomLifecycleNode::on_custom_cleanup,
    test_node.get(), std::placeholders::_1));
  test_node->register_on_shutdown(std::bind(&CustomLifecycleNode::on_custom_shutdown,
    test_node.get(), std::placeholders::_1));
  test_node->register_on_activate(std::bind(&CustomLifecycleNode::on_custom_activate,
    test_node.get(), std::placeholders::_1));
  test_node->register_on_deactivate(std::bind(&CustomLifecycleNode::on_custom_deactivate,
    test_node.get(), std::placeholders::_1));

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
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());

  // check if all callbacks were successfully overwritten
  EXPECT_EQ(5u, test_node->number_of_callbacks);
}
