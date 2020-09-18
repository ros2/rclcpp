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

#include "test_msgs/msg/empty.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

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
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    publisher_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>>(
      get_node_base_interface().get(), std::string("topic"), rclcpp::QoS(10), options);
    add_publisher_handle(publisher_);

    // For coverage this is being added here
    auto timer = create_wall_timer(std::chrono::seconds(1), []() {});
    add_timer_handle(timer);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>> publisher()
  {
    return publisher_;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<test_msgs::msg::Empty>> publisher_;
};

class TestLifecyclePublisher : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<EmptyLifecycleNode>("node");
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<EmptyLifecycleNode> node_;
};

TEST_F(TestLifecyclePublisher, publish) {
  node_->publisher()->on_deactivate();
  EXPECT_FALSE(node_->publisher()->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(node_->publisher()->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(node_->publisher()->publish(std::move(msg_ptr)));
  }
  node_->publisher()->on_activate();
  EXPECT_TRUE(node_->publisher()->is_activated());
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(node_->publisher()->publish(*msg_ptr));
  }
  {
    auto msg_ptr = std::make_unique<test_msgs::msg::Empty>();
    EXPECT_NO_THROW(node_->publisher()->publish(std::move(msg_ptr)));
  }
}
