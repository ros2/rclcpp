// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "rcl/types.h"

#include "rclcpp/subscription_traits.hpp"


void raw_callback_copy(rcl_message_raw_t /* raw_copy */)
{}

void raw_callback_shared_ptr(std::shared_ptr<rcl_message_raw_t> /* raw_shared_ptr */)
{}

void not_raw_callback(char * /* char_ptr */)
{}

void not_raw_shared_ptr_callback(std::shared_ptr<char> /* shared_char_ptr */)
{}

TEST(TestSubscriptionTraits, is_raw_callback) {
  // Test regular functions
  auto cb1 = &raw_callback_copy;
  static_assert(
    rclcpp::subscription_traits::is_raw_callback<decltype(cb1)>::value == true,
    "rcl_message_raw_t in a first argument callback makes it a raw callback");

  auto cb2 = &raw_callback_shared_ptr;
  static_assert(
    rclcpp::subscription_traits::is_raw_callback<decltype(cb2)>::value == true,
    "std::shared_ptr<rcl_message_raw_t> in a first argument callback makes it a raw callback");

  auto cb3 = &not_raw_callback;
  static_assert(
    rclcpp::subscription_traits::is_raw_callback<decltype(cb3)>::value == false,
    "passing a char * is not a raw callback");

  auto cb4 = &not_raw_shared_ptr_callback;
  static_assert(
    rclcpp::subscription_traits::is_raw_callback<decltype(cb4)>::value == false,
    "passing a std::shared_tr<char> is not a raw callback");
}

TEST(TestSubscriptionTraits, callback_messages)
{
  auto cb1 = &raw_callback_copy;
  static_assert(
    std::is_same<
      rcl_message_raw_t,
      rclcpp::subscription_traits::has_message_type<decltype(cb1)>::type>::value,
    "raw callback message type is rcl_message_raw_t");

  auto cb2 = &raw_callback_shared_ptr;
  static_assert(
    std::is_same<
      rcl_message_raw_t,
      rclcpp::subscription_traits::has_message_type<decltype(cb2)>::type>::value,
    "raw callback message type is rcl_message_raw_t");

  auto cb3 = &not_raw_callback;
  static_assert(
    std::is_same<
      char *,
      rclcpp::subscription_traits::has_message_type<decltype(cb3)>::type>::value,
    "not raw callback message type is char");

  auto cb4 = &not_raw_shared_ptr_callback;
  static_assert(
    std::is_same<
      char,
      rclcpp::subscription_traits::has_message_type<decltype(cb4)>::type>::value,
    "not raw shared_ptr callback message type is std::shared_ptr<char>");
}
