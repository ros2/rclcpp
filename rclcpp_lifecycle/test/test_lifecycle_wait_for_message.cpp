// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "test_msgs/msg/strings.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace std::chrono_literals;

class TestLifecycleWaitForMessage : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    node_name_ = std::string("wait_for_message_lifecycle_") + info->name();
    topic_name_ = std::string("wait_for_message_lifecycle_topic_") + info->name();
    // unique_ptr: demonstrate shared_from_this is not required.
    node_ = std::make_unique<rclcpp_lifecycle::LifecycleNode>(node_name_);
  }

  void TearDown() override
  {
    if (node_) {
      node_->shutdown();
      node_.reset();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string node_name_;
  std::string topic_name_;
  std::unique_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

TEST_F(TestLifecycleWaitForMessage, wait_for_message_with_lifecycle_node_interfaces)
{
  using MsgT = test_msgs::msg::Strings;
  auto pub = node_->create_publisher<MsgT>(topic_name_, 10);
  pub->on_activate();

  MsgT out;
  auto received = false;
  auto wait = std::async(
    [&]() {
      // LifecycleNode has no Node::SharedPtr overload; use explicit interfaces.
      auto ret = rclcpp::wait_for_message(
        out,
        node_->get_node_parameters_interface(),
        node_->get_node_topics_interface(),
        topic_name_,
        5s);
      EXPECT_TRUE(ret);
      received = true;
    });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }

  ASSERT_NO_THROW(wait.get());
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);
}
