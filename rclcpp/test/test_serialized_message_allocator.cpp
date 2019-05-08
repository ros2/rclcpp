// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rcl/types.h"

#include "test_msgs/msg/empty.hpp"

TEST(TestSerializedMessageAllocator, default_allocator) {
  using DummyMessageT = float;
  auto mem_strategy =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<DummyMessageT>::create_default();

  auto msg0 = mem_strategy->borrow_serialized_message();
  ASSERT_EQ(msg0->buffer_capacity, 0u);
  mem_strategy->return_serialized_message(msg0);

  auto msg100 = mem_strategy->borrow_serialized_message(100);
  ASSERT_EQ(msg100->buffer_capacity, 100u);
  mem_strategy->return_serialized_message(msg100);

  auto msg200 = mem_strategy->borrow_serialized_message();
  auto ret = rmw_serialized_message_resize(msg200.get(), 200);
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(0u, msg200->buffer_length);
  EXPECT_EQ(200u, msg200->buffer_capacity);
  mem_strategy->return_serialized_message(msg200);

  auto msg1000 = mem_strategy->borrow_serialized_message(1000);
  ASSERT_EQ(msg1000->buffer_capacity, 1000u);
  ret = rmw_serialized_message_resize(msg1000.get(), 2000);
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(2000u, msg1000->buffer_capacity);
  mem_strategy->return_serialized_message(msg1000);
}

TEST(TestSerializedMessageAllocator, borrow_from_subscription) {
  rclcpp::init(0, NULL);

  auto node = std::make_shared<rclcpp::Node>("test_serialized_message_allocator_node");
  std::shared_ptr<rclcpp::SubscriptionBase> sub =
    node->create_subscription<test_msgs::msg::Empty>("~/dummy_topic", 10,
      [](std::shared_ptr<test_msgs::msg::Empty> test_msg) {(void) test_msg;});

  auto msg0 = sub->create_serialized_message();
  EXPECT_EQ(0u, msg0->buffer_capacity);
  sub->return_serialized_message(msg0);

  rclcpp::shutdown();
}
