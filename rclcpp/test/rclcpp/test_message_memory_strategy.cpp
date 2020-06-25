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

#include <memory>

#include "rclcpp/message_memory_strategy.hpp"
#include "test_msgs/msg/empty.hpp"

TEST(TestMemoryStrategies, construct_destruct) {
  rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty> memory_strategy1;

  auto allocator = std::make_shared<std::allocator<void>>();
  rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty> memory_strategy2(
    allocator);
}

TEST(TestMemoryStrategies, standard_allocation) {
  rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty> memory_strategy;
  auto default_message = memory_strategy.create_default();
  ASSERT_NE(nullptr, default_message);

  auto borrowed_message = memory_strategy.borrow_message();
  ASSERT_NE(nullptr, borrowed_message);
  EXPECT_NO_THROW(memory_strategy.return_message(borrowed_message));

  auto serialized_message = memory_strategy.borrow_serialized_message();
  ASSERT_NE(nullptr, serialized_message);
  EXPECT_EQ(0u, serialized_message->capacity());
  EXPECT_NO_THROW(memory_strategy.return_serialized_message(serialized_message));

  memory_strategy.set_default_buffer_capacity(42);
  serialized_message = memory_strategy.borrow_serialized_message();
  ASSERT_NE(nullptr, serialized_message);
  EXPECT_EQ(42u, serialized_message->capacity());
  EXPECT_NO_THROW(memory_strategy.return_serialized_message(serialized_message));
}
