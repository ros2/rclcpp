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

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "test_msgs/msg/empty.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

class TestMessagePoolMemoryStrategy : public ::testing::Test
{
public:
  void SetUp()
  {
    message_memory_strategy_ =
      std::make_shared<MessagePoolMemoryStrategy<test_msgs::msg::Empty, 1>>();
  }

protected:
  std::shared_ptr<MessagePoolMemoryStrategy<test_msgs::msg::Empty, 1>> message_memory_strategy_;
};

TEST_F(TestMessagePoolMemoryStrategy, construct_destruct) {
  ASSERT_NE(nullptr, message_memory_strategy_);
  EXPECT_NE(nullptr, message_memory_strategy_->message_allocator_);
  EXPECT_NE(nullptr, message_memory_strategy_->serialized_message_allocator_);
  EXPECT_NE(nullptr, message_memory_strategy_->buffer_allocator_);
}

TEST_F(TestMessagePoolMemoryStrategy, borrow_return) {
  auto message = message_memory_strategy_->borrow_message();
  ASSERT_NE(nullptr, message);

  EXPECT_NO_THROW(message_memory_strategy_->return_message(message));
}

TEST_F(TestMessagePoolMemoryStrategy, borrow_too_many) {
  auto message = message_memory_strategy_->borrow_message();
  ASSERT_NE(nullptr, message);

  // Size is 1, borrowing second time should fail
  RCLCPP_EXPECT_THROW_EQ(
    message_memory_strategy_->borrow_message(),
    std::runtime_error("No more free slots in the pool"));
  EXPECT_NO_THROW(message_memory_strategy_->return_message(message));
}

TEST_F(TestMessagePoolMemoryStrategy, borrow_hold_reference) {
  {
    auto message = message_memory_strategy_->borrow_message();
    ASSERT_NE(nullptr, message);

    // Return it.
    EXPECT_NO_THROW(message_memory_strategy_->return_message(message));

    // But we are still holding the reference, so we expect that there is still no room in the pool.
    RCLCPP_EXPECT_THROW_EQ(
      message_memory_strategy_->borrow_message(),
      std::runtime_error("No more free slots in the pool"));
  }

  // Now that we've dropped the reference (left the scope), we expect to be able to borrow again.

  auto message2 = message_memory_strategy_->borrow_message();
  ASSERT_NE(nullptr, message2);

  EXPECT_NO_THROW(message_memory_strategy_->return_message(message2));
}
