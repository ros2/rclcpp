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


#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

class TestIntraProcessSubscriber : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions();
    node = std::make_shared<rclcpp::Node>("my_node", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

/*
   Construtctor
 */
TEST_F(TestIntraProcessSubscriber, constructor) {
  using MessageT = test_msgs::msg::Empty;
  using Alloc = std::allocator<void>;

  rclcpp::AnySubscriptionCallback<MessageT, Alloc> callback;

  auto allocator = std::make_shared<Alloc>();
  auto subscriber = std::make_shared<rclcpp::experimental::SubscriptionIntraProcess<MessageT,
      MessageT>>(
    callback,
    allocator,
    rclcpp::contexts::get_global_default_context(),
    "topic",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    rclcpp::IntraProcessBufferType::SharedPtr);

  EXPECT_NE(nullptr, subscriber);
}

/*
   Test methods of SubscriptionIntraProcess with shared callback:
    * is_serialized
    * deserialize_message
    * serve_serialized_message
 */
TEST_F(TestIntraProcessSubscriber, methods_shared) {
  using MessageT = test_msgs::msg::Empty;
  using Alloc = std::allocator<void>;

  // create IPM subscriber
  uint32_t counter = 0u;
  rclcpp::AnySubscriptionCallback<MessageT, Alloc> callback;
  callback.set(
    [&counter](const std::shared_ptr<const MessageT> &) {
      ++counter;
    });

  auto allocator = std::make_shared<Alloc>();
  auto context = rclcpp::contexts::get_global_default_context();
  auto subscriber = std::make_shared<rclcpp::experimental::SubscriptionIntraProcess<MessageT,
      MessageT>>(
    callback,
    allocator,
    context,
    "topic",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    rclcpp::IntraProcessBufferType::SharedPtr);

  EXPECT_NE(nullptr, subscriber);
  EXPECT_EQ(false, subscriber->is_serialized());

  // generate a serialized message
  MessageT msg;
  rclcpp::SerializedMessage serialized_message;
  rclcpp::Serialization<MessageT> serializer;
  serializer.serialize_message(&msg, &serialized_message);

  // test deserialization
  auto deserialized_msg = subscriber->deserialize_message(&serialized_message);
  EXPECT_EQ(msg, *deserialized_msg);

  // Get the intra process manager instance for this context.
  auto ipm = context->get_sub_context<rclcpp::experimental::IntraProcessManager>();
  const auto sub_id = ipm->add_subscription(subscriber);
  auto publisher = node->create_publisher<MessageT>("topic", 42);
  const auto intra_process_publisher_id = ipm->add_publisher(publisher);

  std::vector<uint64_t> take_ownership_subscriptions;
  std::vector<uint64_t> take_shared_subscriptions = {sub_id};
  subscriber->serve_serialized_message(
    &serialized_message,
    ipm.get(),
    intra_process_publisher_id,
    reinterpret_cast<void *>(allocator.get()),
    take_ownership_subscriptions,
    take_shared_subscriptions);

  // execute subscriber callback
  {
    auto data = subscriber->take_data();
    subscriber->execute(data);
  }

  // check if message was received
  EXPECT_EQ(1u, counter);

  // add a 2nd shared subscriber to cover more code
  auto subscriber2 = std::make_shared<rclcpp::experimental::SubscriptionIntraProcess<MessageT,
      MessageT>>(
    callback,
    allocator,
    context,
    "topic",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    rclcpp::IntraProcessBufferType::SharedPtr);
  const auto sub2_id = ipm->add_subscription(subscriber2);
  take_shared_subscriptions.push_back(sub2_id);

  subscriber->serve_serialized_message(
    &serialized_message,
    ipm.get(),
    intra_process_publisher_id,
    reinterpret_cast<void *>(allocator.get()),
    take_ownership_subscriptions,
    take_shared_subscriptions);

  // execute subscribers callback
  {
    auto data = subscriber->take_data();
    subscriber->execute(data);
  }

  // check if message was received
  EXPECT_EQ(2u, counter);

  {
    auto data = subscriber2->take_data();
    subscriber2->execute(data);
  }

  // check if message was received
  EXPECT_EQ(3u, counter);
}

/*
   Test methods of SubscriptionIntraProcess with unique callback:
    * is_serialized
    * deserialize_message
    * serve_serialized_message
 */
TEST_F(TestIntraProcessSubscriber, methods_unique) {
  using MessageT = test_msgs::msg::Empty;
  using Alloc = std::allocator<void>;

  // create IPM subscriber
  uint32_t counter = 0u;
  rclcpp::AnySubscriptionCallback<MessageT, Alloc> callback;
  callback.set(
    [&counter](std::unique_ptr<MessageT>) {
      ++counter;
    });

  auto allocator = std::make_shared<Alloc>();
  auto context = rclcpp::contexts::get_global_default_context();
  auto subscriber = std::make_shared<rclcpp::experimental::SubscriptionIntraProcess<MessageT,
      MessageT>>(
    callback,
    allocator,
    context,
    "topic",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    rclcpp::IntraProcessBufferType::UniquePtr);

  EXPECT_NE(nullptr, subscriber);
  EXPECT_EQ(false, subscriber->is_serialized());

  // generate a serialized message
  MessageT msg;
  rclcpp::SerializedMessage serialized_message;
  rclcpp::Serialization<MessageT> serializer;
  serializer.serialize_message(&msg, &serialized_message);

  // test deserialization
  auto deserialized_msg = subscriber->deserialize_message(&serialized_message);
  EXPECT_EQ(msg, *deserialized_msg);

  // Get the intra process manager instance for this context.
  auto ipm = context->get_sub_context<rclcpp::experimental::IntraProcessManager>();
  const auto sub_id = ipm->add_subscription(subscriber);
  auto publisher = node->create_publisher<MessageT>("topic", 42);
  const auto intra_process_publisher_id = ipm->add_publisher(publisher);

  std::vector<uint64_t> take_ownership_subscriptions = {sub_id};
  std::vector<uint64_t> take_shared_subscriptions;
  subscriber->serve_serialized_message(
    &serialized_message,
    ipm.get(),
    intra_process_publisher_id,
    reinterpret_cast<void *>(allocator.get()),
    take_ownership_subscriptions,
    take_shared_subscriptions);

  // execute subscriber callback
  {
    auto data = subscriber->take_data();
    subscriber->execute(data);
  }

  // check if message was received
  EXPECT_EQ(1u, counter);
}
