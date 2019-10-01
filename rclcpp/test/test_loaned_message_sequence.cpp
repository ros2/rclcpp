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

#include "rclcpp/loaned_message_sequence.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/basic_types.hpp"

using MessageT = test_msgs::msg::BasicTypes;
using LoanedMessageSequenceT = rclcpp::LoanedMessageSequence;

class TestLoanedMessageSequence : public ::testing::Test
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

#define TEST_FOR_EXCEPTION(EXE, EXCEPTION_TYPE) \
  try { \
    EXE \
    FAIL() << "no exception is thrown"; \
  } catch (const EXCEPTION_TYPE & e) { \
    fprintf(stderr, "correct exception thrown: %s\n", e.what()); \
  } catch (...) { \
    fprintf(stderr, "something else got caught\n"); \
    FAIL() << "no exception of type " #EXCEPTION_TYPE " is thrown"; \
  } \

TEST_F(TestLoanedMessageSequence, initialize) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_sequence_test_node");
  auto sub = node->create_subscription<MessageT>(
    "loaned_message_sequence_test_topic", 1, [](const MessageT::SharedPtr msg){(void) msg;});

  LoanedMessageSequenceT loaned_message_sequence(sub.get());
  EXPECT_EQ(0u, loaned_message_sequence.size());
  EXPECT_EQ(0u, loaned_message_sequence.capacity());
  TEST_FOR_EXCEPTION(loaned_message_sequence.at(0);, std::out_of_range);
}

TEST_F(TestLoanedMessageSequence, wrongly_initialized) {
  TEST_FOR_EXCEPTION(LoanedMessageSequenceT lms(nullptr);, std::runtime_error);
}
