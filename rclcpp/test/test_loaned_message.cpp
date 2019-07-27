// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/basic_types.hpp"

using MessageT = test_msgs::msg::BasicTypes;
using LoanedMessageT = rclcpp::LoanedMessage<MessageT>;

TEST(TestLoanedMessage, initialize) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  try {
    auto loaned_msg_ptr = rclcpp::LoanedMessage<MessageT>::get_instance(pub.get());
    ASSERT_TRUE(loaned_msg_ptr->is_valid());
    loaned_msg_ptr->get().float32_value = 42.0f;
    ASSERT_EQ(42.0f, loaned_msg_ptr->get().float32_value);
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  } catch (...) {
    FAIL() << "something bad happened";
  }

  rclcpp::shutdown();
  SUCCEED();
}

TEST(TestLoanedMessage, wrong_initialized) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");

  try {
    auto loaned_msg_ptr = rclcpp::LoanedMessage<MessageT>::get_instance(nullptr);
  } catch (const std::runtime_error & e) {
    SUCCEED();
  } catch (...) {
    FAIL() << "something bad happened";
  }

  rclcpp::shutdown();
}
