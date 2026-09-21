// Copyright 2026 Open Source Robotics Foundation, Inc.
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
#include <variant>

#include "rcl/guard_condition.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/utilities.hpp"

#include "test_msgs/msg/empty.hpp"

#include "../mocking_utils/patch.hpp"

class TestSubscriptionIntraProcess : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  using MessageT = test_msgs::msg::Empty;
  using SubscriptionIntraProcessT = rclcpp::experimental::SubscriptionIntraProcess<
    MessageT, MessageT>;

  static SubscriptionIntraProcessT::SharedPtr
  create_subscription(rclcpp::IntraProcessBufferType buffer_type)
  {
    rclcpp::AnySubscriptionCallback<MessageT> callback;
    callback.set([](std::unique_ptr<MessageT>) {});
    auto allocator = std::make_shared<std::allocator<void>>();
    return SubscriptionIntraProcessT::make_shared(
      callback,
      allocator,
      rclcpp::contexts::get_global_default_context(),
      "topic",
      rclcpp::QoS(10),
      buffer_type);
  }
};

// Regression test for a bug where take_data() checked
// `data.message.index() == std::variant_npos` to detect an empty buffer, but variant_npos
// only occurs for a valueless-by-exception variant -- a default-constructed Data (what an
// empty buffer's consume() returns) holds a null pointer in its default alternative instead,
// so the check never fired and take_data() returned a non-null wrapper around empty data.
TEST_F(TestSubscriptionIntraProcess, take_data_returns_nullptr_when_buffer_is_empty)
{
  auto sub = create_subscription(rclcpp::IntraProcessBufferType::UniquePtr);
  auto data = sub->take_data();
  EXPECT_EQ(nullptr, data);
}

TEST_F(TestSubscriptionIntraProcess, take_data_returns_data_when_available)
{
  auto sub = create_subscription(rclcpp::IntraProcessBufferType::UniquePtr);

  rmw_message_info_t message_info{};
  std::variant<SubscriptionIntraProcessT::MessageUniquePtr,
    SubscriptionIntraProcessT::ConstMessageSharedPtr> message =
    std::make_unique<MessageT>();
  sub->provide_intra_process_message(std::move(message), message_info);

  auto data = sub->take_data();
  EXPECT_NE(nullptr, data);
}

// Regression test for a bug where provide_intra_process_message() called
// trigger_guard_condition() before buffer_->add(), leaving a window where a consumer woken by
// the guard condition could find the buffer still empty. Patches rcl_trigger_guard_condition
// to observe whether the data was already visible (via the public is_ready()) at the moment
// the guard condition actually fires.
TEST_F(TestSubscriptionIntraProcess, provide_intra_process_message_adds_data_before_triggering)
{
  auto sub = create_subscription(rclcpp::IntraProcessBufferType::UniquePtr);
  rcl_wait_set_t dummy_wait_set{};

  bool had_data_when_triggered = false;
  auto record_order = [&sub, &dummy_wait_set, &had_data_when_triggered](
    const rcl_guard_condition_t *) {
      had_data_when_triggered = sub->is_ready(dummy_wait_set);
      return RCL_RET_OK;
    };
  auto mock = mocking_utils::patch(
    "lib:rclcpp", rcl_trigger_guard_condition, record_order);

  rmw_message_info_t message_info{};
  std::variant<SubscriptionIntraProcessT::MessageUniquePtr,
    SubscriptionIntraProcessT::ConstMessageSharedPtr> message =
    std::make_unique<MessageT>();
  sub->provide_intra_process_message(std::move(message), message_info);

  EXPECT_TRUE(had_data_when_triggered);
}
