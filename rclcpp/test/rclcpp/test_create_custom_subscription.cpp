// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <type_traits>

#include "rclcpp/subscription.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/node.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

using namespace std::chrono_literals;

class TestCreateSubscription : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
  typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    ROSMessageT,
    AllocatorT
  >>
class CustomSubscription : public rclcpp::Subscription<
    MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>
{
public:
  template<typename ... Args>
  explicit CustomSubscription(Args &&... args)
  : rclcpp::Subscription<
      MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>(
      std::forward<Args>(args)...) {}
};

TEST_F(TestCreateSubscription, create) {
  using MessageT = test_msgs::msg::Empty;

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](MessageT::ConstSharedPtr) {};

  using CallbackT = std::decay_t<decltype(callback)>;
  using AllocatorT = std::allocator<void>;
  using SubscriptionT = CustomSubscription<MessageT, AllocatorT>;
  using CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<CallbackT>::type;
  using MessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>;

  auto subscription = rclcpp::create_subscription<
    MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
    node, "topic_name", qos, std::move(callback), options);

  ASSERT_NE(nullptr, subscription);
  EXPECT_STREQ("/ns/topic_name", subscription->get_topic_name());
  static_assert(std::is_same_v<std::decay_t<decltype(*subscription.get())>, SubscriptionT>);
}
