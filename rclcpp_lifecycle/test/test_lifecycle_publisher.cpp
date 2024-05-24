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

class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmptyLifecycleNode(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {
    // For coverage this is being added here
    auto timer = create_wall_timer(std::chrono::seconds(1), []() {});
    add_timer_handle(timer);
  }
};

class TestLifecyclePublisher : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
<<<<<<< HEAD
    node_ = std::make_shared<EmptyLifecycleNode>("node");
=======
>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

<<<<<<< HEAD
TEST_F(TestLifecyclePublisher, publish_managed_by_node) {
=======
TEST_P(TestLifecyclePublisher, publish_managed_by_node) {
  auto node = std::make_shared<EmptyLifecycleNode>("node", GetParam());

  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>> publisher =
    node->create_publisher<test_msgs::msg::Empty>(std::string("topic"), rclcpp::QoS(10), options);

>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
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
<<<<<<< HEAD
  node_->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;
  EXPECT_FALSE(node_->publisher()->is_activated());
=======
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
>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
<<<<<<< HEAD
}

TEST_F(TestLifecyclePublisher, publish) {
=======
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

>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
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
<<<<<<< HEAD
  node_->publisher()->on_activate();
  EXPECT_TRUE(node_->publisher()->is_activated());
=======
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
  publisher->on_activate();
  EXPECT_TRUE(publisher->is_activated());
>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(publisher->publish(std::move(msg_ptr)));
  }
<<<<<<< HEAD
=======
  {
    auto loaned_msg = publisher->borrow_loaned_message();
    EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
  }
>>>>>>> 22df1d59 (rclcpp::shutdown should not be called before LifecycleNode dtor. (#2527))
}
