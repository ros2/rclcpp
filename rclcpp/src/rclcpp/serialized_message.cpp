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

#include "rclcpp/serialized_message.hpp"

#include <cstring>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

#include "rmw/types.h"

namespace rclcpp
{

/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
SerializedMessage::SerializedMessage(const rcl_allocator_t & allocator)
: SerializedMessage(0u, allocator)
{}

SerializedMessage::SerializedMessage(
  size_t initial_capacity, const rcl_allocator_t & allocator)
: rcl_serialized_message_t(rmw_get_zero_initialized_serialized_message())
{
  const auto ret = rmw_serialized_message_init(
    this, initial_capacity, &allocator);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

SerializedMessage::SerializedMessage(const SerializedMessage & serialized_message)
: SerializedMessage(static_cast<const rcl_serialized_message_t &>(serialized_message))
{}

SerializedMessage::SerializedMessage(const rcl_serialized_message_t & serialized_message)
: rcl_serialized_message_t(rmw_get_zero_initialized_serialized_message())
{
  fprintf(stderr, "copy constructor called\n");
  const auto ret = rmw_serialized_message_init(
    this, serialized_message.buffer_capacity, &serialized_message.allocator);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // do not call memcpy if the pointer is "static"
  if (buffer != serialized_message.buffer) {
    std::memcpy(buffer, serialized_message.buffer, serialized_message.buffer_length);
  }
  buffer_length = serialized_message.buffer_length;
}

SerializedMessage::SerializedMessage(SerializedMessage && serialized_message)
: SerializedMessage(
    std::forward<rcl_serialized_message_t>(
      static_cast<rcl_serialized_message_t &&>(serialized_message)))
{}

SerializedMessage::SerializedMessage(rcl_serialized_message_t && serialized_message)
: rcl_serialized_message_t(serialized_message)
{
  // reset buffer to prevent double free
  serialized_message = rmw_get_zero_initialized_serialized_message();
}

SerializedMessage & SerializedMessage::operator=(const SerializedMessage & other)
{
  *this = static_cast<const rcl_serialized_message_t &>(other);

  return *this;
}

SerializedMessage & SerializedMessage::operator=(const rcl_serialized_message_t & other)
{
  *this = static_cast<SerializedMessage>(rmw_get_zero_initialized_serialized_message());

  const auto ret = rmw_serialized_message_init(
    this, other.buffer_capacity, &other.allocator);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // do not call memcpy if the pointer is "static"
  if (buffer != other.buffer) {
    std::memcpy(buffer, other.buffer, other.buffer_length);
  }
  buffer_length = other.buffer_length;

  return *this;
}

SerializedMessage & SerializedMessage::operator=(SerializedMessage && other)
{
  *this = static_cast<rcl_serialized_message_t &&>(other);

  return *this;
}

SerializedMessage & SerializedMessage::operator=(rcl_serialized_message_t && other)
{
  this->buffer = other.buffer;
  this->buffer_capacity = other.buffer_capacity;
  this->buffer_length = other.buffer_length;
  this->allocator = other.allocator;

  // reset original to prevent double free
  other = rmw_get_zero_initialized_serialized_message();

  return *this;
}

SerializedMessage::~SerializedMessage()
{
  if (nullptr != buffer) {
    const auto fini_ret = rmw_serialized_message_fini(this);
    if (fini_ret != RCL_RET_OK) {
      RCLCPP_ERROR(
        get_logger("rclcpp"),
        "Failed to destroy serialized message: %s", rcl_get_error_string().str);
    }
  }
}

}  // namespace rclcpp
