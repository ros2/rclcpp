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

TEST(TestAnySubscriptionCallback, unset_dispatch_throw) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  EXPECT_THROW(cb.dispatch(msg, info), std::runtime_error);
  EXPECT_THROW(cb.dispatch_intra_process(const_msg, info), std::runtime_error);
  EXPECT_THROW(cb.dispatch_intra_process(std::move(unique_msg), info), std::runtime_error);
}

TEST(TestAnySubscriptionCallback, set_dispatch_shared_ptr) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto shared_ptr_callback = [&callback_count](
    const std::shared_ptr<test_msgs::msg::Empty>) {
      callback_count++;
    };

  cb.set(std::forward<decltype(shared_ptr_callback)>(shared_ptr_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  // Can't convert ConstSharedPtr to SharedPtr
  EXPECT_THROW(cb.dispatch_intra_process(const_msg, info), std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Promotes Unique into SharedPtr
  EXPECT_NO_THROW(cb.dispatch_intra_process(std::move(unique_msg), info));
  EXPECT_EQ(callback_count, 2);
}

TEST(TestAnySubscriptionCallback, set_dispatch_shared_ptr_w_info) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto shared_ptr_w_info_callback = [&callback_count](
    const std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  cb.set(std::forward<decltype(shared_ptr_w_info_callback)>(shared_ptr_w_info_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  // Can't convert ConstSharedPtr to SharedPtr
  EXPECT_THROW(cb.dispatch_intra_process(const_msg, info), std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Promotes Unique into SharedPtr
  EXPECT_NO_THROW(cb.dispatch_intra_process(std::move(unique_msg), info));
  EXPECT_EQ(callback_count, 2);
}

TEST(TestAnySubscriptionCallback, set_dispatch_const_shared_ptr) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto const_shared_ptr_callback = [&callback_count](
    std::shared_ptr<const test_msgs::msg::Empty>) {
      callback_count++;
    };

  cb.set(std::forward<decltype(const_shared_ptr_callback)>(const_shared_ptr_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  // Ok to promote shared_ptr to ConstSharedPtr
  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  EXPECT_NO_THROW(cb.dispatch_intra_process(const_msg, info));
  EXPECT_EQ(callback_count, 2);

  // Not allowed to convert unique_ptr to const shared_ptr
  EXPECT_THROW(cb.dispatch_intra_process(std::move(unique_msg), info), std::runtime_error);
  EXPECT_EQ(callback_count, 2);
}

TEST(TestAnySubscriptionCallback, set_dispatch_const_shared_ptr_w_info) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto const_shared_ptr_callback = [&callback_count](
    std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  cb.set(std::forward<decltype(const_shared_ptr_callback)>(const_shared_ptr_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  // Ok to promote shared_ptr to ConstSharedPtr
  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  EXPECT_NO_THROW(cb.dispatch_intra_process(const_msg, info));
  EXPECT_EQ(callback_count, 2);

  // Not allowed to convert unique_ptr to const shared_ptr
  EXPECT_THROW(cb.dispatch_intra_process(std::move(unique_msg), info), std::runtime_error);
  EXPECT_EQ(callback_count, 2);
}

TEST(TestAnySubscriptionCallback, set_dispatch_unique_ptr) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto unique_ptr_callback = [&callback_count](
    std::unique_ptr<test_msgs::msg::Empty>) {
      callback_count++;
    };

  cb.set(std::forward<decltype(unique_ptr_callback)>(unique_ptr_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  // Message is copied into unique_ptr
  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  EXPECT_THROW(cb.dispatch_intra_process(const_msg, info), std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Unique_ptr is_moved
  EXPECT_NO_THROW(cb.dispatch_intra_process(std::move(unique_msg), info));
  EXPECT_EQ(callback_count, 2);
}

TEST(TestAnySubscriptionCallback, set_dispatch_unique_ptr_w_info) {
  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty, std::allocator<void>> cb(allocator);

  int callback_count = 0;
  auto unique_ptr_callback = [&callback_count](
    std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {
      callback_count++;
    };

  cb.set(std::forward<decltype(unique_ptr_callback)>(unique_ptr_callback));

  auto msg = std::make_shared<test_msgs::msg::Empty>();
  auto const_msg = std::make_shared<const test_msgs::msg::Empty>();
  auto unique_msg = std::make_unique<test_msgs::msg::Empty>();
  rclcpp::MessageInfo info;

  // Message is copied into unique_ptr
  EXPECT_NO_THROW(cb.dispatch(msg, info));
  EXPECT_EQ(callback_count, 1);

  EXPECT_THROW(cb.dispatch_intra_process(const_msg, info), std::runtime_error);
  EXPECT_EQ(callback_count, 1);

  // Unique_ptr is_moved
  EXPECT_NO_THROW(cb.dispatch_intra_process(std::move(unique_msg), info));
  EXPECT_EQ(callback_count, 2);
}
