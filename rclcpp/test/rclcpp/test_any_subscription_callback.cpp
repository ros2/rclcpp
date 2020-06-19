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

#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/any_subscription_callback.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

class TestAnySubscriptionCallback : public ::testing::Test
{
public:
  TestAnySubscriptionCallback()
  : any_subscription_callback_(allocator_) {}
  void SetUp()
  {
    msg_shared_ptr_ = std::make_shared<test_msgs::msg::Empty>();
    msg_const_shared_ptr_ = std::make_shared<const test_msgs::msg::Empty>();
    msg_unique_ptr_ = std::make_unique<test_msgs::msg::Empty>();
  }

protected:
  std::shared_ptr<std::allocator<void>> allocator_;
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>>
  any_subscription_callback_;

  std::shared_ptr<test_msgs::msg::Empty> msg_shared_ptr_;
  std::shared_ptr<const test_msgs::msg::Empty> msg_const_shared_ptr_;
  std::unique_ptr<test_msgs::msg::Empty> msg_unique_ptr_;
  rclcpp::MessageInfo message_info_;
};

TEST_F(TestAnySubscriptionCallback, construct_destruct) {
}

TEST_F(TestAnySubscriptionCallback, unset_dispatch_throw) {
  EXPECT_THROW(
    any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_),
    std::runtime_error);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_shared_ptr) {
  int callback_count = 0;
  auto shared_ptr_callback = [&callback_count](
    const std::shared_ptr<test_msgs::msg::Empty>) {
      callback_count++;
    };

  any_subscription_callback_.set(shared_ptr_callback);
  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  // Can't convert ConstSharedPtr to SharedPtr
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Promotes Unique into SharedPtr
  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_));
  EXPECT_EQ(callback_count, 2);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_shared_ptr_w_info) {
  int callback_count = 0;
  auto shared_ptr_w_info_callback = [&callback_count](
    const std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  any_subscription_callback_.set(shared_ptr_w_info_callback);

  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  // Can't convert ConstSharedPtr to SharedPtr
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Promotes Unique into SharedPtr
  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_));
  EXPECT_EQ(callback_count, 2);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_const_shared_ptr) {
  int callback_count = 0;
  auto const_shared_ptr_callback = [&callback_count](
    std::shared_ptr<const test_msgs::msg::Empty>) {
      callback_count++;
    };

  any_subscription_callback_.set(const_shared_ptr_callback);

  // Ok to promote shared_ptr to ConstSharedPtr
  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 2);

  // Not allowed to convert unique_ptr to const shared_ptr
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 2);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_const_shared_ptr_w_info) {
  int callback_count = 0;
  auto const_shared_ptr_callback = [&callback_count](
    std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  any_subscription_callback_.set(
    std::move(const_shared_ptr_callback));

  // Ok to promote shared_ptr to ConstSharedPtr
  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 2);

  // Not allowed to convert unique_ptr to const shared_ptr
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 2);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_unique_ptr) {
  int callback_count = 0;
  auto unique_ptr_callback = [&callback_count](
    std::unique_ptr<test_msgs::msg::Empty>) {
      callback_count++;
    };

  any_subscription_callback_.set(unique_ptr_callback);

  // Message is copied into unique_ptr
  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Unique_ptr is_moved
  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_));
  EXPECT_EQ(callback_count, 2);
}

TEST_F(TestAnySubscriptionCallback, set_dispatch_unique_ptr_w_info) {
  int callback_count = 0;
  auto unique_ptr_callback = [&callback_count](
    std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  any_subscription_callback_.set(unique_ptr_callback);

  // Message is copied into unique_ptr
  EXPECT_NO_THROW(any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_));
  EXPECT_EQ(callback_count, 1);

  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_const_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Unique_ptr is_moved
  EXPECT_NO_THROW(
    any_subscription_callback_.dispatch_intra_process(std::move(msg_unique_ptr_), message_info_));
  EXPECT_EQ(callback_count, 2);
}
