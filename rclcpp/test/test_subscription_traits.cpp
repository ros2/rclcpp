// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "rcl/types.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/subscription_traits.hpp"

#include "test_msgs/msg/empty.hpp"

void serialized_callback_copy(rcl_serialized_message_t unused)
{
  (void) unused;
}

void serialized_callback_shared_ptr(std::shared_ptr<rcl_serialized_message_t> unused)
{
  (void) unused;
}

void not_serialized_callback(char * unused)
{
  (void) unused;
}

void not_serialized_shared_ptr_callback(std::shared_ptr<char> unused)
{
  (void) unused;
}

void not_serialized_unique_ptr_callback(
  test_msgs::msg::Empty::UniquePtrWithDeleter<rclcpp::allocator::Deleter<std::allocator<void>,
  test_msgs::msg::Empty>> unused)
{
  (void) unused;
}

TEST(TestSubscriptionTraits, is_serialized_callback) {
  // Test regular functions
  auto cb1 = &serialized_callback_copy;
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb1)>::value == true,
    "rcl_serialized_message_t in a first argument callback makes it a serialized callback");

  auto cb2 = &serialized_callback_shared_ptr;
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb2)>::value == true,
    "std::shared_ptr<rcl_serialized_message_t> in a callback makes it a serialized callback");

  auto cb3 = &not_serialized_callback;
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb3)>::value == false,
    "passing a char * is not a serialized callback");

  auto cb4 = &not_serialized_shared_ptr_callback;
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb4)>::value == false,
    "passing a std::shared_tr<char> is not a serialized callback");

  auto cb5 = [](rcl_serialized_message_t unused) -> void
    {
      (void) unused;
    };
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb5)>::value == true,
    "rcl_serialized_message_t in a first argument callback makes it a serialized callback");

  using MessageT = test_msgs::msg::Empty;
  using MessageTAllocator = std::allocator<void>;
  using MessageTDeallocator = rclcpp::allocator::Deleter<MessageTAllocator, MessageT>;
  auto cb6 = [](MessageT::UniquePtrWithDeleter<MessageTDeallocator> unique_msg_ptr) -> void
    {
      (void) unique_msg_ptr;
    };
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb6)>::value == false,
    "passing a std::unique_ptr of test_msgs::msg::Empty is not a serialized callback");

  auto cb7 = &not_serialized_unique_ptr_callback;
  static_assert(
    rclcpp::subscription_traits::is_serialized_callback<decltype(cb7)>::value == false,
    "passing a fancy unique_ptr of test_msgs::msg::Empty is not a serialized callback");
}

TEST(TestSubscriptionTraits, callback_messages) {
  static_assert(
    std::is_same<
      std::shared_ptr<char>,
      rclcpp::function_traits::function_traits<
        decltype(not_serialized_shared_ptr_callback)
      >::template argument_type<0>
    >::value, "wrong!");

  static_assert(
    std::is_same<
      char,
      rclcpp::subscription_traits::extract_message_type<
        rclcpp::function_traits::function_traits<
          decltype(not_serialized_shared_ptr_callback)
        >::template argument_type<0>
      >::type
    >::value, "wrong!");

  auto cb1 = &serialized_callback_copy;
  static_assert(
    std::is_same<
      rcl_serialized_message_t,
      rclcpp::subscription_traits::has_message_type<decltype(cb1)>::type>::value,
    "serialized callback message type is rcl_serialized_message_t");

  auto cb2 = &serialized_callback_shared_ptr;
  static_assert(
    std::is_same<
      rcl_serialized_message_t,
      rclcpp::subscription_traits::has_message_type<decltype(cb2)>::type>::value,
    "serialized callback message type is rcl_serialized_message_t");

  auto cb3 = &not_serialized_callback;
  static_assert(
    std::is_same<
      char *,
      rclcpp::subscription_traits::has_message_type<decltype(cb3)>::type>::value,
    "not serialized callback message type is char");

  auto cb4 = &not_serialized_shared_ptr_callback;
  static_assert(
    std::is_same<
      char,
      rclcpp::subscription_traits::has_message_type<decltype(cb4)>::type>::value,
    "not serialized shared_ptr callback message type is std::shared_ptr<char>");

  auto cb5 = [](rcl_serialized_message_t unused) -> void
    {
      (void) unused;
    };
  static_assert(
    std::is_same<
      rcl_serialized_message_t,
      rclcpp::subscription_traits::has_message_type<decltype(cb5)>::type>::value,
    "serialized callback message type is rcl_serialized_message_t");

  using MessageT = test_msgs::msg::Empty;
  using MessageTAllocator = std::allocator<MessageT>;
  using MessageTDeallocator = rclcpp::allocator::Deleter<MessageTAllocator, MessageT>;
  auto cb6 = [](std::unique_ptr<MessageT, MessageTDeallocator> unique_msg_ptr) -> void
    {
      (void) unique_msg_ptr;
    };
  static_assert(
    std::is_same<
      test_msgs::msg::Empty,
      rclcpp::subscription_traits::has_message_type<decltype(cb6)>::type>::value,
    "passing a std::unique_ptr of test_msgs::msg::Empty has message type Empty");

  auto cb7 = &not_serialized_unique_ptr_callback;
  static_assert(
    std::is_same<
      test_msgs::msg::Empty,
      rclcpp::subscription_traits::has_message_type<decltype(cb7)>::type>::value,
    "passing a fancy std::unique_ptr of test_msgs::msg::Empty has message type Empty");
}
