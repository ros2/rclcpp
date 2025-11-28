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
#include <string>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "test_msgs/msg/empty.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class TestDefaultStateMachine : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

/// We want to test everything for both the wall and generic timer.
enum class TimerType
{
  WALL_TIMER,
  GENERIC_TIMER,
};

class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmptyLifecycleNode(const std::string & node_name, const TimerType & timer_type)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {
    // For coverage this is being added here
    switch (timer_type) {
      case TimerType::WALL_TIMER:
        {
          auto timer = create_wall_timer(std::chrono::seconds(1), []() {});
          add_timer_handle(timer);
          break;
        }
      case TimerType::GENERIC_TIMER:
        {
          auto timer = create_timer(std::chrono::seconds(1), []() {});
          add_timer_handle(timer);
          break;
        }
    }
  }
};

class TestLifecyclePublisher : public ::testing::TestWithParam<TimerType>
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_P(TestLifecyclePublisher, publish_managed_by_node) {
  auto node = std::make_shared<EmptyLifecycleNode>("node", GetParam());

  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>> publisher =
    node->create_publisher<test_msgs::msg::Empty>(std::string("topic"), rclcpp::QoS(10), options);

  // transition via LifecycleNode
  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto reset_key = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  auto ret = reset_key;

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());
  node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  EXPECT_TRUE(publisher->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
  node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  (void)ret;  // Just to make clang happy
  EXPECT_FALSE(publisher->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
}

TEST_P(TestLifecyclePublisher, publish) {
  auto node = std::make_shared<EmptyLifecycleNode>("node", GetParam());

  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>> publisher =
    node->create_publisher<test_msgs::msg::Empty>(std::string("topic"), rclcpp::QoS(10), options);

  // transition via LifecyclePublisher
  publisher->on_deactivate();
  EXPECT_FALSE(publisher->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
  publisher->on_activate();
  EXPECT_TRUE(publisher->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
}

INSTANTIATE_TEST_SUITE_P(
  PerTimerType, TestLifecyclePublisher,
  ::testing::Values(TimerType::WALL_TIMER, TimerType::GENERIC_TIMER),
  [](const ::testing::TestParamInfo<TimerType> & info) -> std::string {
    switch (info.param) {
      case TimerType::WALL_TIMER:
        return std::string("wall_timer");
      case TimerType::GENERIC_TIMER:
        return std::string("generic_timer");
      default:
        break;
    }
    return std::string("unknown");
  }
);
