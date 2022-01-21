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
#include <utility>

#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/basic_types.hpp"

#include "../mocking_utils/patch.hpp"

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

// suppress deprecated function warning
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

  auto pub_allocator = pub->get_allocator();

// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

// suppress deprecated function warning
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

  auto loaned_msg = rclcpp::LoanedMessage<MessageT>(pub.get(), pub_allocator);

// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

  ASSERT_TRUE(loaned_msg.is_valid());
  loaned_msg.get().float32_value = 42.0f;
  ASSERT_EQ(42.0f, loaned_msg.get().float32_value);

  SUCCEED();
}

TEST_F(TestLoanedMessage, loan_from_pub) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  auto loaned_msg = pub->borrow_loaned_message();
  ASSERT_TRUE(loaned_msg.is_valid());
  loaned_msg.get().float64_value = 42.0f;
  ASSERT_EQ(42.0f, loaned_msg.get().float64_value);

  SUCCEED();
}

TEST_F(TestLoanedMessage, release) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  std::unique_ptr<MessageT, std::function<void(MessageT *)>> msg;
  {
    auto loaned_msg = pub->borrow_loaned_message();
    ASSERT_TRUE(loaned_msg.is_valid());
    loaned_msg.get().float64_value = 42.0f;
    ASSERT_EQ(42.0f, loaned_msg.get().float64_value);
    msg = loaned_msg.release();
    // call destructor implicitly.
    // destructor not allowed to free memory because of not having ownership
    // of the data after a call to release.
  }

  ASSERT_EQ(42.0f, msg->float64_value);

  // Generally, the memory released from `LoanedMessage::release()` will be freed
  // in deleter of unique_ptr or is managed in the middleware after calling
  // `Publisher::do_loaned_message_publish` inside Publisher::publish().
  if (pub->can_loan_messages()) {
    ASSERT_EQ(
      RCL_RET_OK,
      rcl_return_loaned_message_from_publisher(pub->get_publisher_handle().get(), msg.get()));
  }

  SUCCEED();
}

TEST_F(TestLoanedMessage, construct_with_loaned_message_publisher) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto publisher = node->create_publisher<MessageT>("topic", 10);
  std::allocator<MessageT> allocator;

  auto mock_can_loan = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_publisher_can_loan_messages, true);

  {
    auto mock_borrow_loaned = mocking_utils::patch_and_return(
      "self", rcl_borrow_loaned_message, RCL_RET_ERROR);

    EXPECT_THROW(
      std::make_shared<LoanedMessageT>(*publisher.get(), allocator).reset(),
      rclcpp::exceptions::RCLError);
  }

  MessageT message;
  auto borrow_loaned_message_callback =
    [&message](
    const rcl_publisher_t *, const rosidl_message_type_support_t *, void ** ros_message) {
      *ros_message = &message;
      return RCL_RET_OK;
    };
  auto mock_borrow_loaned = mocking_utils::patch(
    "self", rcl_borrow_loaned_message, borrow_loaned_message_callback);

  {
    auto mock_return_loaned = mocking_utils::patch_and_return(
      "self", rcl_return_loaned_message_from_publisher, RCL_RET_OK);

    auto loaned_message = std::make_shared<LoanedMessageT>(*publisher.get(), allocator);
    EXPECT_TRUE(loaned_message->is_valid());
    EXPECT_NO_THROW(loaned_message.reset());
  }

  {
    auto loaned_message = std::make_shared<LoanedMessageT>(*publisher.get(), allocator);
    EXPECT_TRUE(loaned_message->is_valid());

    auto mock_return_loaned = mocking_utils::patch_and_return(
      "self", rcl_return_loaned_message_from_publisher, RCL_RET_ERROR);

    // No exception, it just logs an error
    EXPECT_NO_THROW(loaned_message.reset());
  }
}

TEST_F(TestLoanedMessage, move_loaned_message) {
  auto node = std::make_shared<rclcpp::Node>("loaned_message_test_node");
  auto pub = node->create_publisher<MessageT>("loaned_message_test_topic", 1);

  auto loaned_msg_to_move = pub->borrow_loaned_message();
  // Force the move constructor to invoke
  auto loaned_msg_moved_to = LoanedMessageT(std::move(loaned_msg_to_move));

  ASSERT_TRUE(loaned_msg_moved_to.is_valid());
  ASSERT_FALSE(loaned_msg_to_move.is_valid());

  loaned_msg_moved_to.get().float32_value = 42.0f;
  ASSERT_EQ(42.0f, loaned_msg_moved_to.get().float32_value);
  SUCCEED();
}
