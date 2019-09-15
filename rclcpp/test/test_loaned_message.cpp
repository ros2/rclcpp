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

class TestLoanedMessage : public ::testing::Test
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

TEST_F(TestLoanedMessage, initialize) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  try {
    auto loaned_msg = rclcpp::LoanedMessage<MessageT>::get_instance(pub.get());
    ASSERT_TRUE(loaned_msg.is_valid());
    loaned_msg.get().float32_value = 42.0f;
    ASSERT_EQ(42.0f, loaned_msg.get().float32_value);
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  } catch (...) {
    FAIL() << "something bad happened";
  }

  SUCCEED();
}

TEST_F(TestLoanedMessage, wrong_initialized) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");

  try {
    auto loaned_msg = rclcpp::LoanedMessage<MessageT>::get_instance(nullptr);
  } catch (const std::runtime_error & e) {
    SUCCEED();
  } catch (...) {
    FAIL() << "something bad happened";
  }
}

TEST_F(TestLoanedMessage, loan_from_pub) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  try {
    auto loaned_msg = pub->loan_message();
    ASSERT_TRUE(loaned_msg.is_valid());
    loaned_msg.get().float64_value = 42.0f;
    ASSERT_EQ(42.0f, loaned_msg.get().float64_value);
  } catch (const std::runtime_error & e) {
    FAIL() << e.what();
  }

  SUCCEED();
}
