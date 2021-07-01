// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "test_msgs/msg/strings.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace std::chrono_literals;

TEST(TestUtilities, wait_for_message) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("wait_for_message_node");

  using MsgT = test_msgs::msg::Strings;
  auto pub = node->create_publisher<MsgT>("wait_for_message_topic", 10);

  MsgT out;
  auto received = false;
  auto wait = std::async([&]() {
      auto ret = rclcpp::wait_for_message(out, node, "wait_for_message_topic", 5s);
      EXPECT_TRUE(ret);
      received = true;
      });

  for (auto i = 0u; i < 10 && received == false; ++i) {
    pub->publish(*get_messages_strings()[0]);
    std::this_thread::sleep_for(1s);
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(out, *get_messages_strings()[0]);
}
