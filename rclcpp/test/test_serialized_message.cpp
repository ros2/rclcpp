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
  EXPECT_TRUE(serialized_message.buffer == nullptr);
  EXPECT_EQ(0u, serialized_message.buffer_length);
  EXPECT_EQ(0u, serialized_message.buffer_capacity);
}

TEST(TestSerializedMessage, initialize_with_capacity) {
  rclcpp::SerializedMessage serialized_message(13);
  EXPECT_TRUE(serialized_message.buffer != nullptr);
  EXPECT_EQ(0u, serialized_message.buffer_length);
  EXPECT_EQ(13u, serialized_message.buffer_capacity);
}

TEST(TestSerializedMessage, various_constructors) {
  const std::string content = "Hello World";
  const auto content_size = content.size() + 1;  // accounting for null terminator

  rclcpp::SerializedMessage serialized_message(content_size);
  // manually copy some content
  std::memcpy(serialized_message.buffer, content.c_str(), content.size());
  serialized_message.buffer[content.size()] = '\0';
  serialized_message.buffer_length = content_size;
  EXPECT_STREQ(content.c_str(), reinterpret_cast<char *>(serialized_message.buffer));
  EXPECT_EQ(content_size, serialized_message.buffer_capacity);

  // Copy Constructor
  rclcpp::SerializedMessage other_serialized_message(serialized_message);
  EXPECT_EQ(content_size, other_serialized_message.buffer_capacity);
  EXPECT_EQ(content_size, other_serialized_message.buffer_length);
  EXPECT_STREQ(
    reinterpret_cast<char *>(serialized_message.buffer),
    reinterpret_cast<char *>(other_serialized_message.buffer));

  // Move Constructor
  rclcpp::SerializedMessage yet_another_serialized_message(std::move(other_serialized_message));
  EXPECT_TRUE(other_serialized_message.buffer == nullptr);
  EXPECT_EQ(0u, other_serialized_message.buffer_capacity);
  EXPECT_EQ(0u, other_serialized_message.buffer_length);

  auto default_allocator = rcl_get_default_allocator();
  auto rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto yet_another_rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&rcl_serialized_msg, 13, &default_allocator);
  ASSERT_EQ(RCL_RET_OK, ret);
  ret = rmw_serialized_message_init(&yet_another_rcl_serialized_msg, 13, &default_allocator);
  ASSERT_EQ(RCL_RET_OK, ret);

  // manually copy some content
  std::memcpy(rcl_serialized_msg.buffer, content.c_str(), content.size());
  rcl_serialized_msg.buffer[content.size()] = '\0';
  rcl_serialized_msg.buffer_length = content_size;
  EXPECT_EQ(13u, rcl_serialized_msg.buffer_capacity);
  std::memcpy(yet_another_rcl_serialized_msg.buffer, content.c_str(), content.size());
  yet_another_rcl_serialized_msg.buffer[content.size()] = '\0';
  yet_another_rcl_serialized_msg.buffer_length = content_size;
  EXPECT_EQ(13u, yet_another_rcl_serialized_msg.buffer_capacity);

  // Copy Constructor from rcl_serialized_message_t
  rclcpp::SerializedMessage from_rcl_msg(rcl_serialized_msg);
  EXPECT_EQ(13u, from_rcl_msg.buffer_capacity);
  EXPECT_EQ(content_size, from_rcl_msg.buffer_length);

  // Move Constructor from rcl_serialized_message_t
  rclcpp::SerializedMessage yet_another_moved_serialized_message(std::move(
      yet_another_rcl_serialized_msg));
  EXPECT_TRUE(yet_another_rcl_serialized_msg.buffer == nullptr);
  EXPECT_EQ(0u, yet_another_rcl_serialized_msg.buffer_capacity);
  EXPECT_EQ(0u, yet_another_rcl_serialized_msg.buffer_length);

  // Verify that despite being fini'd, the copy is real
  ret = rmw_serialized_message_fini(&rcl_serialized_msg);
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(nullptr, rcl_serialized_msg.buffer);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_capacity);
  EXPECT_EQ(0u, rcl_serialized_msg.buffer_length);
  EXPECT_TRUE(nullptr != from_rcl_msg.buffer);
  EXPECT_EQ(13u, from_rcl_msg.buffer_capacity);
  EXPECT_EQ(content_size, from_rcl_msg.buffer_length);
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

template<typename MessageT>
void test_empty_msg_serialize()
{
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;
  rclcpp::SerializedMessage serialized_msg;

  serializer.serialize_message(&ros_msg, &serialized_msg);
}

template<typename MessageT>
void test_empty_msg_deserialize()
{
  rclcpp::Serialization<MessageT> serializer;
  MessageT ros_msg;
  rclcpp::SerializedMessage serialized_msg;

  serializer.deserialize_message(&serialized_msg, &ros_msg);
}
