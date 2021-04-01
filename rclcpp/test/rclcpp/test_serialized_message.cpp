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

#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcpputils/asserts.hpp"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/basic_types.hpp"

TEST(TestSerializedMessage, empty_initialize) {
  rclcpp::SerializedMessage serialized_message;
  EXPECT_EQ(0u, serialized_message.size());
  EXPECT_EQ(0u, serialized_message.capacity());
}

TEST(TestSerializedMessage, initialize_with_capacity) {
  rclcpp::SerializedMessage serialized_message(13);
  EXPECT_EQ(0u, serialized_message.size());
  EXPECT_EQ(13u, serialized_message.capacity());
}

TEST(TestSerializedMessage, various_constructors) {
  const std::string content = "Hello World";
  const auto content_size = content.size() + 1;  // accounting for null terminator

  rclcpp::SerializedMessage serialized_message(content_size);
  // manually copy some content
  auto & rcl_handle = serialized_message.get_rcl_serialized_message();
  std::memcpy(rcl_handle.buffer, content.c_str(), content.size());
  rcl_handle.buffer[content.size()] = '\0';
  rcl_handle.buffer_length = content_size;
  EXPECT_STREQ(content.c_str(), reinterpret_cast<char *>(rcl_handle.buffer));
  EXPECT_EQ(content_size, serialized_message.capacity());
  EXPECT_EQ(content_size, serialized_message.size());

  // Copy Constructor
  rclcpp::SerializedMessage other_serialized_message(serialized_message);
  EXPECT_EQ(content_size, other_serialized_message.capacity());
  EXPECT_EQ(content_size, other_serialized_message.size());
  auto & other_rcl_handle = other_serialized_message.get_rcl_serialized_message();
  EXPECT_STREQ(
    reinterpret_cast<char *>(rcl_handle.buffer),
    reinterpret_cast<char *>(other_rcl_handle.buffer));

  // Move Constructor
  rclcpp::SerializedMessage yet_another_serialized_message(std::move(other_serialized_message));
  auto & yet_another_rcl_handle = yet_another_serialized_message.get_rcl_serialized_message();
  EXPECT_TRUE(nullptr == other_rcl_handle.buffer);
  EXPECT_EQ(0u, other_serialized_message.capacity());
  EXPECT_EQ(0u, other_serialized_message.size());
  EXPECT_TRUE(nullptr != yet_another_rcl_handle.buffer);
  EXPECT_EQ(content_size, yet_another_serialized_message.size());
  EXPECT_EQ(content_size, yet_another_serialized_message.capacity());
}

TEST(TestSerializedMessage, various_constructors_from_rcl) {
  const std::string content = "Hello World";
  const auto content_size = content.size() + 1;  // accounting for null terminator

  auto default_allocator = rcl_get_default_allocator();

  auto rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&rcl_serialized_msg, 13, &default_allocator);
  ASSERT_EQ(RCL_RET_OK, ret);
  // manually copy some content
  std::memcpy(rcl_serialized_msg.buffer, content.c_str(), content.size());
  rcl_serialized_msg.buffer[content.size()] = '\0';
  rcl_serialized_msg.buffer_length = content_size;
  EXPECT_EQ(13u, rcl_serialized_msg.buffer_capacity);

  // Copy Constructor from rcl_serialized_message_t
  rclcpp::SerializedMessage serialized_message(rcl_serialized_msg);
  EXPECT_EQ(13u, serialized_message.capacity());
  EXPECT_EQ(content_size, serialized_message.size());

  // Move Constructor from rcl_serialized_message_t
  rclcpp::SerializedMessage another_serialized_message(std::move(rcl_serialized_msg));
  EXPECT_TRUE(nullptr == rcl_serialized_msg.buffer);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_capacity);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_length);
  EXPECT_EQ(13u, another_serialized_message.capacity());
  EXPECT_EQ(content_size, another_serialized_message.size());

  // Verify that despite being fini'd, the copy is real
  ret = rmw_serialized_message_fini(&rcl_serialized_msg);
  ASSERT_EQ(RCUTILS_RET_INVALID_ARGUMENT, ret);  // Buffer is null, because it was moved
  EXPECT_EQ(nullptr, rcl_serialized_msg.buffer);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_capacity);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_length);
  EXPECT_EQ(13u, serialized_message.capacity());
  EXPECT_EQ(content_size, serialized_message.size());

  auto rcl_handle = serialized_message.get_rcl_serialized_message();
  EXPECT_TRUE(nullptr != rcl_handle.buffer);
}

TEST(TestSerializedMessage, release) {
  const std::string content = "Hello World";
  const auto content_size = content.size() + 1;  // accounting for null terminator

  rcl_serialized_message_t released_handle = rmw_get_zero_initialized_serialized_message();
  {
    rclcpp::SerializedMessage serialized_msg(13);
    // manually copy some content
    auto & rcl_serialized_msg = serialized_msg.get_rcl_serialized_message();
    std::memcpy(rcl_serialized_msg.buffer, content.c_str(), content.size());
    rcl_serialized_msg.buffer[content.size()] = '\0';
    rcl_serialized_msg.buffer_length = content_size;
    EXPECT_EQ(13u, serialized_msg.capacity());

    released_handle = serialized_msg.release_rcl_serialized_message();
    // scope exit of serialized_msg
  }

  EXPECT_TRUE(nullptr != released_handle.buffer);
  EXPECT_EQ(13u, released_handle.buffer_capacity);
  EXPECT_EQ(content_size, released_handle.buffer_length);
  // cleanup memory manually
  EXPECT_EQ(RCL_RET_OK, rmw_serialized_message_fini(&released_handle));
}

TEST(TestSerializedMessage, reserve) {
  rclcpp::SerializedMessage serialized_msg(13);
  EXPECT_EQ(13u, serialized_msg.capacity());

  // Resize using reserve method
  serialized_msg.reserve(15);
  EXPECT_EQ(15u, serialized_msg.capacity());

  // Use invalid value throws exception
  EXPECT_THROW(
    {serialized_msg.reserve(-1);},
    rclcpp::exceptions::RCLBadAlloc);
}

TEST(TestSerializedMessage, serialization) {
  using MessageT = test_msgs::msg::BasicTypes;

  rclcpp::Serialization<MessageT> serializer;

  auto basic_type_ros_msgs = get_messages_basic_types();
  for (const auto & ros_msg : basic_type_ros_msgs) {
    // convert ros msg to serialized msg
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(ros_msg.get(), &serialized_msg);

    // convert serialized msg back to ros msg
    MessageT deserialized_ros_msg;
    serializer.deserialize_message(&serialized_msg, &deserialized_ros_msg);

    EXPECT_EQ(*ros_msg, deserialized_ros_msg);
  }
}

TEST(TestSerializedMessage, assignment_operators) {
  const std::string content = "Hello World";
  const auto content_size = content.size() + 1;  // accounting for null terminator
  auto default_allocator = rcl_get_default_allocator();
  auto rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&rcl_serialized_msg, 13, &default_allocator);
  ASSERT_EQ(RCL_RET_OK, ret);

  // manually copy some content
  std::memcpy(rcl_serialized_msg.buffer, content.c_str(), content_size);
  rcl_serialized_msg.buffer_length = content_size;
  EXPECT_EQ(13u, rcl_serialized_msg.buffer_capacity);
  rclcpp::SerializedMessage serialized_message_to_assign(rcl_serialized_msg);
  EXPECT_EQ(13u, serialized_message_to_assign.capacity());
  EXPECT_EQ(content_size, serialized_message_to_assign.size());

  // Test copy assignment with = operator, on another rclcpp::SerializedMessage
  rclcpp::SerializedMessage serialized_msg_copy(2);
  EXPECT_EQ(2u, serialized_msg_copy.capacity());
  EXPECT_EQ(0u, serialized_msg_copy.size());
  serialized_msg_copy = serialized_message_to_assign;
  EXPECT_EQ(13u, serialized_msg_copy.capacity());
  EXPECT_EQ(content_size, serialized_msg_copy.size());

  // Test copy assignment with = operator, with a rcl_serialized_message_t
  rclcpp::SerializedMessage serialized_msg_copy_rcl(2);
  EXPECT_EQ(2u, serialized_msg_copy_rcl.capacity());
  EXPECT_EQ(0u, serialized_msg_copy_rcl.size());
  serialized_msg_copy_rcl = rcl_serialized_msg;
  EXPECT_EQ(13u, serialized_msg_copy_rcl.capacity());
  EXPECT_EQ(content_size, serialized_msg_copy_rcl.size());

  // Test move assignment with = operator
  rclcpp::SerializedMessage serialized_msg_move(2);
  EXPECT_EQ(2u, serialized_msg_move.capacity());
  EXPECT_EQ(0u, serialized_msg_move.size());
  serialized_msg_move = std::move(serialized_message_to_assign);
  EXPECT_EQ(13u, serialized_msg_move.capacity());
  EXPECT_EQ(content_size, serialized_msg_move.size());

  // Test move assignment with = operator, with a rcl_serialized_message_t
  rclcpp::SerializedMessage serialized_msg_move_rcl(2);
  EXPECT_EQ(2u, serialized_msg_move_rcl.capacity());
  EXPECT_EQ(0u, serialized_msg_move_rcl.size());
  serialized_msg_move_rcl = std::move(rcl_serialized_msg);
  EXPECT_EQ(13u, serialized_msg_move_rcl.capacity());
  EXPECT_EQ(content_size, serialized_msg_move_rcl.size());

  // Error because it was moved
  EXPECT_EQ(RCUTILS_RET_INVALID_ARGUMENT, rmw_serialized_message_fini(&rcl_serialized_msg));
}

TEST(TestSerializedMessage, failed_init_throws) {
  rclcpp::SerializedMessage serialized_msg(13);
  EXPECT_EQ(13u, serialized_msg.capacity());

  // Constructor with invalid size throws exception
  EXPECT_THROW(
    {rclcpp::SerializedMessage serialized_msg_fail(-1);},
    rclcpp::exceptions::RCLBadAlloc);

  // Constructor copy with rmw_serialized bad msg throws
  auto default_allocator = rcl_get_default_allocator();
  auto rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&rcl_serialized_msg, 13, &default_allocator);
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(13u, rcl_serialized_msg.buffer_capacity);
  rcl_serialized_msg.buffer_capacity = -1;
  EXPECT_THROW(
    {rclcpp::SerializedMessage serialized_msg_fail_2(rcl_serialized_msg);},
    rclcpp::exceptions::RCLBadAlloc);

  rcl_serialized_msg.buffer_capacity = 13;
  EXPECT_EQ(RCL_RET_OK, rmw_serialized_message_fini(&rcl_serialized_msg));
}

void serialize_default_ros_msg()
{
  using MessageT = test_msgs::msg::BasicTypes;
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;
  rclcpp::SerializedMessage serialized_msg;

  serializer.serialize_message(&ros_msg, &serialized_msg);
}

void serialize_default_ros_msg_into_nullptr()
{
  using MessageT = test_msgs::msg::BasicTypes;
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;

  serializer.serialize_message(&ros_msg, nullptr);
}

void deserialize_default_serialized_message()
{
  using MessageT = test_msgs::msg::BasicTypes;
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;
  rclcpp::SerializedMessage serialized_msg;

  serializer.deserialize_message(&serialized_msg, &ros_msg);
}

void deserialize_nullptr()
{
  using MessageT = test_msgs::msg::BasicTypes;
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;
  rclcpp::SerializedMessage serialized_msg;

  serializer.deserialize_message(&serialized_msg, &ros_msg);
}

TEST(TestSerializedMessage, serialization_empty_messages)
{
  EXPECT_NO_THROW(serialize_default_ros_msg());
  EXPECT_THROW(serialize_default_ros_msg_into_nullptr(), rcpputils::IllegalStateException);
  EXPECT_THROW(serialize_default_ros_msg_into_nullptr(), rcpputils::IllegalStateException);
  EXPECT_THROW(deserialize_default_serialized_message(), rcpputils::IllegalStateException);
  EXPECT_THROW(deserialize_nullptr(), rcpputils::IllegalStateException);
}
